#!/usr/bin/env python

# importamos este modulo que nos permite obtener la lista de argumentos de la linea de comandos
import sys
# Importamos la libreria cliente ROS para Python
import rospy
# Importamos la declaracion de servicio "ObjectPoseService" y los mensajes de solicitud y respuesta
from unity_robotics_demo_msgs.srv import ObjectPoseService, ObjectPoseServiceRequest, ObjectPoseServiceResponse

class ObjectPoseClient():
    def __init__(self):
        pass

    # Llamamos al servicio
    def callService(self,  service_name, object_name):
        # Esperamos hasta que el servicio este disponible
        rospy.wait_for_service(service_name)
        # Creamos una instancia del servicio (la cual podemos invocar)
        self.__srv=rospy.ServiceProxy(service_name, ObjectPoseService)
        # Creamos nuestro objeto de solicitud
        req=ObjectPoseServiceRequest(object_name=object_name)
        # Llamamos al servicio y obtenemos un objeto de respuesta
        res=self.__srv(req)
        rospy.loginfo_once("\nCall made")
        return res

def main():
    if len(sys.argv)==1:
         print("%s [object_name]"%(sys.argv[0]))
         sys.exit(1)
    else:
        # Guardamos el nombre del objeto
        object_name=sys.argv[1]
        # Creamos una instancia de la clase
        srv=ObjectPoseClient()
        # Llamamos al servicio
        res=srv.callService("obj_pose_srv", object_name)
        # El tipo de mensaje de respuesta es de tipo "geometry_msgs/Pose":
        # geometry_msgs/Point position
        #   float64 x
        #   float64 y
        #   float64 z
        # geometry_msgs/Quaternion orientation
        #   float64 x
        #   float64 y
        #   float64 z
        #   float64 w
        print("Request: \n{}".format(res.object_pose))

if __name__=='__main__':
    try:
        main()
    except rospy.ServiceException as exception:
        pass

