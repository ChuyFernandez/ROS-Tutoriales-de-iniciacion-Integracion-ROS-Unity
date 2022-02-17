#!/usr/bin/env python

# importamos este modulo que nos permite obtener la lista de argumentos de la linea de comandos
import sys
# Importamos la clase random para generar numeros aleatorios
import random
# Importamos la libreria cliente ROS para Python
import rospy
# Importamos la declaracion de mensaje "UnityColor"
from unity_robotics_demo_msgs.msg import UnityColor

class ColorPublisher():
    def __init__(self, topic_name):
        # Inicializamos un nodo ROS
        rospy.init_node("color_publisher", anonymous=False)
        # Creamos una instancia para publicar
        self.__publisher=rospy.Publisher(topic_name, UnityColor, queue_size=10)

    # Publicamos un color aleatorio en el tema especificado
    def __publish(self):
        color=UnityColor(r=random.randint(0,255), g=random.randint(0,255), b=random.randint(0,255), a=random.randint(0,255))
        self.__publisher.publish(color)
        rospy.loginfo_once("\nPublishing color")

    # Definimos la manera de publicar
    def publish(self, rate_second=0):
        # Publicar una vez o a una cierta tasa (lo decide el usuario)
        if rate_second==0:
            self.__publish()
        else:
            while not rospy.is_shutdown():
                self.__publish()
                rospy.sleep(rate_second)

def main():
    # Creamos una instancia de la clase
    color_publisher=ColorPublisher(topic_name="color")
    # Guardamos el valor de la entrada al ejecutar el script (si es que el usuario especifico uno)
    rate_second=0 if len(sys.argv)==1 else float(sys.argv[1])
    # Realizamos la tarea de publicar
    color_publisher.publish(rate_second=rate_second)

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

