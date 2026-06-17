import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge
import cv2
from PIL import Image as PILImage
import time
from google import genai
# Aquí importamos el servicio del srv
from gemini_interfaces.srv import VisionQuery 

class VisionGemini(Node):
    def __init__(self):
        super().__init__('gemini_vision')
        
        self.bridge = CvBridge()
        self.ultima_imagen = None
        self.ultimo_fallo_tiempo = 0.0  
        self.tiempo_cooldown = 2.0

        self.client = genai.Client(api_key='AIzaSyA9169Rnraup9Kh64xXSHSUID4Jv3HC2wY')
        
        # Creamos el suscriptor
        self.sub_camara = self.create_subscription(
            ROSImage,
            '/camera/image',
            self.callback_camara,
            1
        )
        
        # Creamos el servicio
        self.srv = self.create_service(
            VisionQuery, 
            '/preguntar_a_gemini', 
            self.peticion_vision_callback
        )
        self.get_logger().info("Puedo ver, esperando una petición...")

    def callback_camara(self, msg):
        """Vamos tomando la información que nos llega por el topic y la escribimos a una variable."""
        self.ultima_imagen = msg

    def peticion_vision_callback(self, request, response):
        """Función principal de visión, se activa solo cuando se envia una peticion al servidor, ya que es su callback."""
        objeto_buscado = request.objeto
        tiempo_actual = time.time()

        if (tiempo_actual - self.ultimo_fallo_tiempo) < self.tiempo_cooldown:
            self.get_logger().warn("Cooldown para no saturar Gemini.")
            response.encontrado = False
            return response

        self.get_logger().info(f"Objeto a buscar: {objeto_buscado}")
        
        if self.ultima_imagen is None:
            self.get_logger().warn("No recibi nada por el topic /camera/image.")
            response.encontrado = False
            return response

        try:
            # Hacemos las transformaciones necesarias para poder enviar la imagen a Gemini

            # Primero transformamos el vector binario que nos llega por ros a una matriz numérica de valores BGR
            cv_img = self.bridge.imgmsg_to_cv2(self.ultima_imagen, "bgr8")
            # Ahora transformamos de BGR a RGB
            rgb_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
            # Por ultimo con la imagen en RGB guardamos en pil_img la imagen en RAW
            pil_img = PILImage.fromarray(rgb_img)
            
            # Llamada a gemini
            self.get_logger().info("Enviando fotograma a Gemini...")
            prompt = f"¿Ves un {request.objeto} en la imagen? Responde solo SI o NO."
            respuesta_gemini = self.client.models.generate_content(
                model='gemini-flash-lite-latest',
                contents=[prompt, pil_img]
            )
            texto_respuesta = respuesta_gemini.text.strip().upper()
            self.get_logger().info(f"Gemini respondió: {texto_respuesta}")
            

            if "SI" in texto_respuesta.upper():
                self.get_logger().info(f"¡Encontré un/una {objeto_buscado.capitalize()}!")
                response.encontrado = True
            else:
                self.get_logger().info(f"No veo ningún/a {objeto_buscado}.")
                response.encontrado = False
                
        except Exception as e:
            self.get_logger().error(f"Error al mandar a Gemini: {e}")
            self.ultimo_fallo_tiempo = time.time() 
            response.encontrado = False
            
        return response

def main(args=None):
    rclpy.init(args=args)
    nodo = VisionGemini()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        nodo.get_logger().info("Apagando visión...")
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()