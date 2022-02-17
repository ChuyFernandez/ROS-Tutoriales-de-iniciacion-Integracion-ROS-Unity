#!/usr/bin/env python

# Importamos la clase random para generar numeros aleatorios
import random
from types import UnionType

from numpy import place
# Importamos la libreria cliente ROS para Python
import rospy
# Importamos la declaracion de servicio "PositionService" y los mensajes de solicitud y respuesta
from unity_robotics_demo_msgs.srv import PositionService, PositionServiceRequest, PositionServiceResponse
# Importamos la declaracion de mensaje "PosRot"
from unity_robotics_demo_msgs.msg import PosRot

class ClassPositionService():
    def __init__(self):
        # Inicializamos un nodo ROS
        rospy.init_node("position_service", anonymous=False)
    
    def createService(self, srv_name):
        # Creamos el servicio
        self.__srv=rospy.Service(srv_name, PositionService, self.__handler)
        rospy.loginfo_once("\nService created")
        # Mantenemos ejecutando el nodo hasta que se le de una senal de apagado
        rospy.spin()
    
    # Recibimos una solicitud y devolvemos una respuesta
    def __handler(self, req):
        # Guardamos la solicitud en una variable (para mas facil acceso)
        posRotReq=req.input
        # Mostramos el contenido de la solicitud
        print("Request: \n{}".format(posRotReq))
        # Cremamos un objeto del tipo de mensaje "PosRot"
        posRotRes=PosRot()
        # La posicion aleatoria en xz corresponde a dentro del plano en la escena de Unity 
        posRotRes.pos_x=random.randint(-4,4)
        posRotRes.pos_y=posRotReq.pos_y
        posRotRes.pos_z=random.randint(-4,4)
        posRotRes.rot_x=posRotReq.rot_x
        posRotRes.rot_y=posRotReq.rot_y
        posRotRes.rot_z=posRotReq.rot_z
        posRotRes.rot_w=posRotReq.rot_w
        # Devolvemos una respuesta (una posicion nueva)
        return PositionServiceResponse(output=posRotRes)

def main():
    # Creamos una instancia de la clase
    position_service=ClassPositionService()
    # Creamos el servicio
    position_service.createService(srv_name="pos_srv")

if __name__=='__main__':
    try:
        main()
    except rospy.ServiceException as exception:
        pass

