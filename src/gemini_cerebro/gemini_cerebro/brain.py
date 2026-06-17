import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import Float64
import speech_recognition as sr
from google import genai
from google.genai import types
import json
import os
import py_trees
import py_trees_ros
import time
from gemini_interfaces.srv import VisionQuery 
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class DespachadorDeTareas(py_trees.behaviour.Behaviour):
    """Extrae la siguiente tarea de la lista y la prepara para su ejecución."""
    def __init__(self, name, nodo_ros):
        super().__init__(name)
        self.nodo_ros = nodo_ros
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key="plan_de_accion", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="accion_actual", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="destino", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="objeto", access=py_trees.common.Access.WRITE)

    def update(self):
        # Comprobamos si hay un plan en la pizarra
        if not self.blackboard.exists("plan_de_accion") or not self.blackboard.plan_de_accion:
            return py_trees.common.Status.FAILURE

        # Leemos la primera tarea de la lista sin borrarla aún
        tarea_actual = self.blackboard.plan_de_accion[0]
        
        # Ponemos los datos en la pizarra para que los lean los demás nodos
        self.blackboard.accion_actual = tarea_actual.get("accion")
        self.blackboard.destino = tarea_actual.get("destino")
        self.blackboard.objeto = tarea_actual.get("objeto")
        
        return py_trees.common.Status.SUCCESS

class FinalizarTarea(py_trees.behaviour.Behaviour):
    """Borra la tarea actual de la lista una vez completada."""
    def __init__(self, name, nodo_ros):
        super().__init__(name)
        self.nodo_ros = nodo_ros
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key="plan_de_accion", access=py_trees.common.Access.WRITE)

    def update(self):
        # Borramos la tarea recién se realize
        if self.blackboard.exists("plan_de_accion") and self.blackboard.plan_de_accion:
            tarea_borrada = self.blackboard.plan_de_accion.pop(0)
            self.nodo_ros.get_logger().info(f"Tarea completada: {tarea_borrada['accion']}")
        return py_trees.common.Status.SUCCESS

class ComprobarAccion(py_trees.behaviour.Behaviour):
    """Devuelve SUCCESS si la accion_actual de la pizarra coincide con la esperada."""
    def __init__(self, name, accion_esperada):
        super().__init__(name)
        self.accion_esperada = accion_esperada
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key="accion_actual", access=py_trees.common.Access.READ)

    def update(self):
        # Leemos la pizarra
        if not self.blackboard.exists("accion_actual"):
            return py_trees.common.Status.FAILURE
            
        # Si la palabra es la que busco, dejo pasar. Si no, bloqueo.
        if self.blackboard.accion_actual == self.accion_esperada:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

class Movimiento(py_trees.behaviour.Behaviour):
    """Nodo de movimiento: Manda el destino a Nav2 con un timeout."""

    def __init__(self, name, nodo_ros, timeout_segundos=120.0):
        super(Movimiento, self).__init__(name)
        self.nodo_ros = nodo_ros
        self.timeout_segundos = timeout_segundos  # Tiempo máximo de timeout
        
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key="destino", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="ultima_ubicacion", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="ultima_ubicacion", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="ubicacion_anterior", access=py_trees.common.Access.WRITE)
        
        # Variables de la accion
        self.action_client = None
        self.goal_future = None
        self.result_future = None
        self.goal_handle = None
        self.tiempo_inicio = 0.0

    def obtener_coordenadas(self):
        """Lee el archivo JSON en tiempo real para obtener las últimas coordenadas."""
        ruta_json = os.path.expanduser('~/tfg_ws/src/contexto/coordenadas.json')
        try:
            with open(ruta_json, 'r', encoding='utf-8') as archivo:
                return json.load(archivo)
        except Exception as e:
            self.nodo_ros.get_logger().error(f"Error al leer coordenadas.json: {e}")
            return {}

    def setup(self, **kwargs):
        """Se ejecuta 1 vez al principio. Conectamos con el hardware/ROS."""
        self.nodo_ros.get_logger().info(f"[{self.name}] Setup: Conectando...")
        self.action_client = ActionClient(self.nodo_ros, NavigateToPose, 'navigate_to_pose')
        return True

    def initialise(self):
        """Se ejecuta cada vez que el nodo se inicializa."""
        self.nodo_ros.get_logger().info(f"[{self.name}] Initialise: Preparando ruta...")
        self.blackboard.ubicacion_anterior = self.blackboard.ultima_ubicacion

        destino_texto = self.blackboard.destino
        self.coordenadas_conocidas = self.obtener_coordenadas()
        
        # Si el destino no está en el mapa, no nos movemos
        if destino_texto not in self.coordenadas_conocidas:
            self.nodo_ros.get_logger().error(f"[{self.name}] Destino desconocido: {destino_texto}.")
            self.goal_future = None
            return
            
        x, y, z, w = self.coordenadas_conocidas[destino_texto]
        self.nodo_ros.get_logger().info(f"[{self.name}] Viaje a {destino_texto}. Límite: {self.timeout_segundos}s.")
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.nodo_ros.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.z = float(z)
        goal_msg.pose.pose.orientation.w = float(w)
        
        self.result_future = None
        self.goal_handle = None
        
        # Registramos el momento exacto en el que empezamos a movernos
        self.tiempo_inicio = time.time()
        self.goal_future = self.action_client.send_goal_async(goal_msg)

    def update(self):
        """Lo que se realiza 10 veces por segundo."""
        if self.goal_future is None:
            return py_trees.common.Status.FAILURE

        # Añadimos timeout a la navegación para que no se bloquee si no alcanza destino
        tiempo_en_ruta = time.time() - self.tiempo_inicio
        if tiempo_en_ruta > self.timeout_segundos:
            self.nodo_ros.get_logger().warn(f"[{self.name}] ¡Tiempo agotado ({self.timeout_segundos}s)! El robot está atascado.")

            # Si hay un goal_handle activo, le pedimos a Nav2 que cancele todo
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
            return py_trees.common.Status.FAILURE

        if self.goal_handle is None:
            if self.goal_future.done():
                self.goal_handle = self.goal_future.result()
                if not self.goal_handle.accepted:
                    self.nodo_ros.get_logger().error(f"[{self.name}] Nav2 ha rechazado la ruta.")
                    return py_trees.common.Status.FAILURE
                # Si se acepta la ruta
                self.result_future = self.goal_handle.get_result_async()
            return py_trees.common.Status.RUNNING

        if not self.result_future.done():
            return py_trees.common.Status.RUNNING

        result = self.result_future.result()
        
        if result.status == 4: # Status 4 es que supera con exito
            self.nodo_ros.get_logger().info(f"[{self.name}] ¡Destino alcanzado!")
            self.blackboard.ultima_ubicacion = self.blackboard.destino
            self.goal_handle = None
            return py_trees.common.Status.SUCCESS
        else:
            self.nodo_ros.get_logger().warn(f"[{self.name}] Nav2 no pudo alcanzar el objetivo. Estado final: {result.status}")
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        """Se ejecuta al terminar o al ser interrumpido."""
        self.nodo_ros.get_logger().info(f"[{self.name}] Terminate: Status.{new_status.name}")
        # Si nos cancelan a mitad de camino y no es un reseteo normal
        if new_status == py_trees.common.Status.INVALID and self.goal_handle is not None:
            self.nodo_ros.get_logger().warn(f"[{self.name}] ¡Me han interrumpido! Cancelando meta en Nav2...")
            self.goal_handle.cancel_goal_async()


class DetectarObjeto(py_trees.behaviour.Behaviour):
    """Nodo de la cámara: Pedimos al nodo de deteccion de imagenes que detecte la imagen que llega desde el topic."""
    
    def __init__(self, name, nodo_ros):
        super(DetectarObjeto, self).__init__(name)
        self.nodo_ros = nodo_ros
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key="objeto", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="coordenadas_objeto", access=py_trees.common.Access.WRITE)

        self.objeto_encontrado = None
        self.cli = None
        self.future = None
        
    def setup(self, **kwargs):
        """Se ejecuta 1 vez al principio. Conectamos con el hardware/ROS."""
        self.nodo_ros.get_logger().info(f"[{self.name}] Setup: Conectando con la camara...")
        # Creamos un cliente para el servicio VisionQuery ofrecido por el nodo de la camara
        self.cli = self.nodo_ros.create_client(
            VisionQuery, 
            '/preguntar_a_gemini'
        )
        if not self.cli.wait_for_service(timeout_sec=2.0):
            self.nodo_ros.get_logger().warn("El nodo de visión no está activo. Esperando...")

        return True

    def initialise(self):
        """Se ejecuta cada vez que el nodo 'despierta' para una nueva misión."""
        self.objeto_a_buscar = self.blackboard.objeto
        self.nodo_ros.get_logger().info(f"[{self.name}] Initialise: Detectando objeto {self.objeto_a_buscar}...")
        self.peticion_enviada = False
        self.future = None


    def update(self):
        """Lo que se realiza 10 veces por segundo."""
        # Creamos una peticion al servidor
        if not self.peticion_enviada:
            req = VisionQuery.Request()
            req.objeto = self.objeto_a_buscar
            
            # Hacemos la llamada asíncrona
            self.future = self.cli.call_async(req)
            self.peticion_enviada = True
            
            return py_trees.common.Status.RUNNING

        # Comprobamos si el servicio ha dado una respuesta
        if not self.future.done():
            return py_trees.common.Status.RUNNING
        
        # Si hay respuesta, la extraemos
        result = self.future.result()
        
        # Comprobamos si ha encontrado o no el objeto_a_buscar
        if result.encontrado == True:
            self.nodo_ros.get_logger().info("¡Encontre el objeto!")
            # Me invento unas coordenadas, aquí comprobaria mediante otro servicio las cordenadas del objeto
            self.blackboard.coordenadas_objeto = [1.0, 0.0, 0.5]
            return py_trees.common.Status.SUCCESS
            
        else:
            self.nodo_ros.get_logger().warn("No encontre el objeto.")
            # Si devuelve False, el nodo falla.
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        """Se ejecuta al terminar."""
        self.nodo_ros.get_logger().info(f"[{self.name}] Terminate: Status.{new_status.name}")
        #if new_status != py_trees.common.Status.SUCCESS and new_status != py_trees.common.Status.INVALID:
        #    self.nodo_ros.get_logger().info("¡Me han interrumpido y no pude buscar el objeto!")

class GirarCuello(py_trees.behaviour.Behaviour):
    """Nodo auxiliar para girar el cuello del robot si no detecta el objeto."""
    def __init__(self, name, nodo_ros, angulo_objetivo, tiempo_espera=6.5):
        super(GirarCuello, self).__init__(name)
        self.nodo_ros = nodo_ros
        self.pub_cuello = None
        self.angulo_objetivo = angulo_objetivo
        self.tiempo_espera = tiempo_espera
        self.tiempo_inicio = 0.0
        
    def setup(self, **kwargs):
        """Se ejecuta 1 vez al principio. Conectamos con el hardware/ROS."""
        self.nodo_ros.get_logger().info(f"[{self.name}] Setup: Inicializando motores del cuello...")
        # Creamos un publisher al topic del cuello
        self.pub_cuello = self.nodo_ros.create_publisher(
            Float64,
            '/comando_cuello',
            10
        )
        return True

    def initialise(self):
        """Se ejecuta cada vez que el nodo 'despierta' para una nueva misión."""
        self.nodo_ros.get_logger().info(f"[{self.name}] Girando cuello a posicion: {self.angulo_objetivo}")
        # Publicamos el valor del angulo
        msg = Float64()
        msg.data = float(self.angulo_objetivo)
        self.pub_cuello.publish(msg)
        
        self.tiempo_inicio = time.time()

    def update(self):
        """Lo que se realiza 10 veces por segundo."""
        tiempo_transcurrido = time.time() - self.tiempo_inicio

        if tiempo_transcurrido < self.tiempo_espera:
            return py_trees.common.Status.RUNNING
        else:
            self.nodo_ros.get_logger().info(f"[{self.name}] Giro completado.")
            return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        """Se ejecuta al terminar."""
        self.nodo_ros.get_logger().info(f"[{self.name}] Terminate: Status.{new_status.name}")
        if new_status != py_trees.common.Status.SUCCESS and new_status != py_trees.common.Status.INVALID:
            self.nodo_ros.get_logger().info("¡No he podido girar el cuello!")

class ObjetivoFallido(py_trees.behaviour.Behaviour):
    """Nodo que limpia la memoria cuando otro nodo fracasa."""
    def __init__(self, name, nodo_ros):
        super(ObjetivoFallido, self).__init__(name)
        self.nodo_ros = nodo_ros
        self.blackboard = py_trees.blackboard.Client(name=name)

        # Acceso a la Blackboard
        self.blackboard.register_key(key="objeto", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="destino", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="accion_actual", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="plan_de_accion", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="ubicacion_anterior", access=py_trees.common.Access.READ)

    def update(self):
        try:
            ubicacion_retorno = self.blackboard.ubicacion_anterior
        except KeyError:
            ubicacion_retorno = "salon"

        self.nodo_ros.get_logger().warn(f"[{self.name}] Nodo fallido. Retornando a: {ubicacion_retorno}")
        
        # Inyectamos la orden de volver hacia el usuario
        self.blackboard.plan_de_accion = [{
            "accion": "navegar",
            "destino": ubicacion_retorno, 
            "objeto": None
        }]
        
        # Limpiamos las variables actuales
        self.blackboard.objeto = None
        self.blackboard.destino = None
        self.blackboard.accion_actual = None
        
        # Devolvemos FAILURE para que la rama aborte
        return py_trees.common.Status.FAILURE

class SoltarObjeto(py_trees.behaviour.Behaviour):
    """Nodo de soltar objeto: No se hace nada, simplemente se dice que se ha logrado. Es un nodo que se podría expandir a futuro, pero no es el objetivo."""
    def __init__(self, name, nodo_ros):
        super(SoltarObjeto, self).__init__(name)
        self.nodo_ros = nodo_ros

    def update(self):
        self.nodo_ros.get_logger().info("¡Objeto depositado con éxito!")
        
        return py_trees.common.Status.SUCCESS

class CogerObjeto(py_trees.behaviour.Behaviour):
    """Nodo de soltar objeto: No se hace nada, simplemente se dice que se ha logrado. Es un nodo que se podría expandir a futuro, pero no es el objetivo."""
    def __init__(self, name, nodo_ros):
        super(CogerObjeto, self).__init__(name)
        self.nodo_ros = nodo_ros

    def update(self):
        self.nodo_ros.get_logger().info("¡Objeto cogido con éxito!")
        
        return py_trees.common.Status.SUCCESS

class EsperarOrdenes(py_trees.behaviour.Behaviour):
    """Nodo inactivo que mantiene el árbol vivo esperando a que el Gemini inyecte un plan."""
    def __init__(self, name):
        super().__init__(name)

    def update(self):

        return py_trees.common.Status.RUNNING

class CerebroGemini(Node):
    def __init__(self):
        super().__init__('gemini_coordinator_node')
        self.get_logger().info("Bienvenido/a, dame un segundo mientras me preparo...")
        
        # Inicializar cliente de Gemini 
        # export GEMINI_API_KEY="AIzaSyA9169Rnraup9Kh64xXSHSUID4Jv3HC2wY"
        self.gemini_client = genai.Client(api_key='AIzaSyA9169Rnraup9Kh64xXSHSUID4Jv3HC2wY')
        
        # Creamos una sesión de chat con memoria con Gemini Flash Lite (Probé con un mejor modelo pero está siempre saturado de usuarios)
        self.chat_session = self.gemini_client.chats.create(
            model='gemini-flash-lite-latest',
            config=types.GenerateContentConfig(
                response_mime_type="application/json", # Mantenemos el formato JSON
                http_options={'timeout': 15000}, # Timeout de 15 segundos por si los servidores van lentos (Pasa muchisimo de 17:00 a 22:00)
            )
        )

        # Inicializar el reconocimiento de voz
        self.recognizer = sr.Recognizer()
        self.mic = sr.Microphone()
        self.avisado_escucha = False

        # Arreglo para el bloqueo del programa por culpa del timer de escuchar audio.
        self.grupo_audio = MutuallyExclusiveCallbackGroup()
        self.grupo_arbol = MutuallyExclusiveCallbackGroup()
        
        # Iniciamos el bucle de escucha de 5s
        self.timer = self.create_timer(5.0, self.ciclo_de_escucha, callback_group=self.grupo_audio)

        # Inicializar el BT
        self.crear_arbol()
        self.timer_bt = self.create_timer(0.1, self.tick_arbol, callback_group=self.grupo_arbol)

        # Inicializar Blackboard
        self.blackboard = py_trees.blackboard.Client(name="Master")
        self.blackboard.register_key(key="destino", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="objeto", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="accion", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="coordenadas_objeto", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="plan_de_accion", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="plan_de_accion", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="ultima_ubicacion", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="ultima_ubicacion", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="ubicacion_anterior", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="ubicacion_anterior", access=py_trees.common.Access.READ)

        # Creamos las variables antes de que se utilizen
        self.blackboard.destino = None # Estado inicial limpio
        self.blackboard.objeto = None # Estado inicial limpio
        self.blackboard.accion = None # Estado inicial limpio
        self.blackboard.coordenadas_objeto = None # Estado inicial limpio
        self.blackboard.ultima_ubicacion = "salon" # Estado inicial predefinido
        self.blackboard.ubicacion_anterior = "salon"

    def crear_arbol(self):
        """Construimos el BT."""
        # Creamos la raíz, es un selector entre dos secuencias, la de acción y la de pensar
        self.raiz = py_trees.composites.Selector(name="Selector_Principal", memory=False)

        # Creamos los selectores o fallback para la camara y la navegacion
        selector_busqueda = py_trees.composites.Selector(name="Estrategia_de_Busqueda", memory=True)
        nodo_navegar = py_trees.composites.Selector(name="Nodo_Navegacion", memory=True)

        # Creamos un nodo de selección de acciones dentro de la secuencia de acciones
        nodo_enrutador = py_trees.composites.Selector("Enrutador_Acciones", memory=False)
        
        # Instanciamos los nodos del BT
        nodo_movimiento = Movimiento(name="Navegacion", nodo_ros=self)
        nodo_nav_fallida = ObjetivoFallido(name="Nodo_Nav_Fallida", nodo_ros=self)
        nodo_fallido = ObjetivoFallido(name="Nodo_fallido", nodo_ros=self)
        nodo_fin_tarea = FinalizarTarea(name="Fin_tarea", nodo_ros=self)
        nodo_despachador = DespachadorDeTareas(name="Nodo_despachador_tareas", nodo_ros=self)
        nodo_coger = CogerObjeto(name="Nodo_coger_objeto", nodo_ros=self)
        nodo_soltar = SoltarObjeto(name="Nodo_soltar_objeto", nodo_ros=self)
        esperando_orden = EsperarOrdenes(name="Esperando...")

        # Instancias de la camara
        nodo_camara = DetectarObjeto(name="Buscar_con_camara", nodo_ros=self)
        nodo_camara_izq = DetectarObjeto(name="Mirar_Izquierda", nodo_ros=self)
        nodo_camara_der = DetectarObjeto(name="Mirar_Derecha", nodo_ros=self)

        # Instancias del cuello 
        nodo_cuello_izq = GirarCuello(name="Girar_Izq", nodo_ros=self, angulo_objetivo=1.57) 
        nodo_cuello_der = GirarCuello(name="Girar_Der", nodo_ros=self, angulo_objetivo=-1.57) 
        nodo_cuello_centro = GirarCuello(name="Centrar_Cuello", nodo_ros=self, angulo_objetivo=0.0)

        #Creamos secuencias o selectores para cada giro de camara
        # Intento Izquierda
        seq_izq = py_trees.composites.Sequence(name="Intento_Izquierda", memory=True)
        seq_izq.add_children([nodo_cuello_izq, nodo_camara_izq])

        # Intento Derecha
        seq_der = py_trees.composites.Sequence(name="Intento_Derecha", memory=True)
        seq_der.add_children([nodo_cuello_der, nodo_camara_der])

        # Intento Rendirse 
        seq_fin = py_trees.composites.Sequence(name="Rendirse", memory=True)
        seq_fin.add_children([nodo_cuello_centro, nodo_fallido])

        # Metemos las acciones dentro del Selector, (esto es el nodo de la camara)
        selector_busqueda.add_children([
            nodo_camara,   # Intento 1: Comprobamos directamente la camara.
            seq_izq,       # Intento 2: Giramos a 90 grados la camara.
            seq_der,       # Intento 3: Giramos a -90 grados la camara.
            seq_fin,       # Intento 4: Giramos a 0 grados la cámara y nos rendimos.
        ])

        # Selector para la navegacion
        nodo_navegar.add_children([nodo_movimiento, nodo_nav_fallida])

        # Secuencia de ejecución de acciones
        seq_acciones = py_trees.composites.Sequence(name="Ejecucion_de_acciones", memory=True)
        seq_acciones.add_children([nodo_despachador, nodo_enrutador, nodo_fin_tarea])

        # Secuencias de acciones posibles para el nodo enrutador
        seq_movimiento = py_trees.composites.Sequence(name="Movimiento", memory=True)
        seq_movimiento.add_children([ComprobarAccion("¿Navegar?", "navegar"), nodo_navegar])

        seq_coger = py_trees.composites.Sequence(name="Coger_Objeto", memory=True)
        seq_coger.add_children([ComprobarAccion("¿Coger?", "coger"), selector_busqueda, nodo_coger])

        seq_soltar = py_trees.composites.Sequence(name="Soltar_Objeto", memory=True)
        seq_soltar.add_children([ComprobarAccion("¿Soltar?", "soltar"), nodo_soltar])

        # Añadimos las secuencias de acciones posibles al nodo enrutador
        nodo_enrutador.add_children([seq_movimiento, seq_coger, seq_soltar])
        
        # Unimos los hijos a la raíz
        self.raiz.add_children([seq_acciones, esperando_orden])
        
        # Preparamos el árbol
        self.arbol = py_trees.trees.BehaviourTree(self.raiz)
        self.arbol.setup(timeout=15)

    def tick_arbol(self):
        """Esto se ejecuta 10 veces por segundo."""
        self.arbol.tick()

    def ciclo_de_escucha(self):
        """Captura audio, lo transcribe y llama a Gemini."""
        # El robot está ocupado si existe el plan y tiene al menos 1 fila pendiente
        if self.blackboard.exists("plan_de_accion"):
            plan_actual = self.blackboard.plan_de_accion
        else:
            plan_actual = []

        robot_ocupado = len(plan_actual) > 0

        if robot_ocupado:
            self.avisado_escucha = False
            return 

        texto_usuario = self.capturar_audio()

        if texto_usuario:
            self.get_logger().info(f"Dijiste: {texto_usuario}")
            orden_estructurada = self.consultar_gemini(texto_usuario)
            
            if orden_estructurada:
                self.ejecutar_orden(orden_estructurada)

    def capturar_audio(self):
        """Graba unos segundos desde el micrófono y lo pasa a texto."""
    
        try:
            with self.mic as source:
                if not self.avisado_escucha:
                    self.get_logger().info("Dame una orden por favor...")
                    self.avisado_escucha = True

                self.recognizer.adjust_for_ambient_noise(source)
                audio = self.recognizer.listen(source, timeout=5, phrase_time_limit=5)
            
            # Usamos el motor de Google para el reconocimiento de audio
            texto = self.recognizer.recognize_google(audio, language="es-ES")
            return texto
            
        except sr.WaitTimeoutError:
            return None
            
        except sr.UnknownValueError:
            if rclpy.ok():
                self.get_logger().warn("Lo siento, escuché ruido pero no entendí palabras.")
            return None
            
        except Exception as e:
            if rclpy.ok():
                self.get_logger().error(f"Error de hardware o red: {e}")
            return None

    def obtener_coordenadas(self):
        """Lee el archivo JSON en tiempo real para obtener las últimas coordenadas."""
        ruta_json = os.path.expanduser('~/tfg_ws/src/contexto/coordenadas.json')
        
        try:
            with open(ruta_json, 'r', encoding='utf-8') as archivo:
                return json.load(archivo)
        except Exception as e:
            self.get_logger().error(f"Error al leer coordenadas.json: {e}")
            return {}

    def consultar_gemini(self, texto):
        """Envia el texto a Gemini."""
        # Cargamos las coordenadas actualizadas
        self.coordenadas_conocidas = self.obtener_coordenadas()
        nombres_destinos = ", ".join(self.coordenadas_conocidas.keys())

        # Construimos la ruta al prompt de texto
        ruta_prompt = os.path.expanduser('~/tfg_ws/src/contexto/prompt_contexto.txt')
        
        try:
            with open(ruta_prompt, 'r', encoding='utf-8') as archivo:
                plantilla_prompt = archivo.read()

            prompt = plantilla_prompt.format(texto=texto, lista_destinos=nombres_destinos)
            
        except Exception as e:
            self.get_logger().error(f"Error leyendo el archivo prompt_contexto.txt: {e}")
            return None

        try:
            response = self.chat_session.send_message(prompt)
            datos = json.loads(response.text)
            self.get_logger().info(f"Órden recibida: {datos}")
            return datos
            
        except Exception as e:
            self.get_logger().error(f"Error al contactar con Gemini o parsear JSON: {e}")
            return None

    def ejecutar_orden(self, orden):
        """Traduce el JSON de Gemini (la lista de pasos) al Blackboard.""" 
        # Extraemos la lista de pasos
        plan = orden.get("plan", [])
        
        # Si el plan está vacío
        if not plan:
            self.get_logger().info("Plan vacío o charla trivial. Sigo esperando órdenes...")
            self.avisado_escucha = False  
            return
            
        # Iteramos por cada paso para limpiar los nulls
        for paso in plan:
            if paso.get("destino") == "null":
                paso["destino"] = None
            if paso.get("objeto") == "null":
                paso["objeto"] = None
                
        # Copiamos el plan en el Blackboard
        self.blackboard.plan_de_accion = plan
        
        # Imprimimos el plan completo en la terminal
        self.get_logger().info(f"¡Blackboard actualizada! Plan inyectado con {len(plan)} paso(s):")
        for i, paso in enumerate(plan):
            accion = paso.get('accion')
            destino = paso.get('destino')
            objeto = paso.get('objeto')
            self.get_logger().info(f"  Paso {i+1} -> Acción: {accion} | Destino: {destino} | Objeto: {objeto}")


def main(args=None):
    rclpy.init(args=args)
    nodo = CerebroGemini()

    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(nodo)
    # Cerramos el programa sin que entre en pánico
    try:
        executor.spin()
    except KeyboardInterrupt:
        nodo.get_logger().info("Apagando Coordinador...")
    finally:
        executor.shutdown()
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()