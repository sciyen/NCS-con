#!/usr/bin/env python2.7
import rospy
import socket
import threading
from ncs_con.msg import Con_msg
from ncs_con.msg import cam_msg
from ncs_con.msg import obj_msg
import json
import time

class NcsconThread():
    def __init__(self, port, pubTopic):
        self.indata = [{} for i in range(5)]
        self.port = port
        self.msg = Con_msg()
        self.pub = rospy.Publisher(pubTopic, Con_msg, queue_size=1)

        cam_num = 2
        obj_num = 3
        for i in range(0,obj_num):
            om = obj_msg()
            for j in range(0,cam_num):
                cm = cam_msg()
                om.cams.append(cm)
            self.msg.objs.append(om)


        in_thread = threading.Thread(target=self.__in_thread)
        in_thread.daemon = True
        in_thread.start()

        pub_thread = threading.Thread(target=self.__pub_thread)
        pub_thread.daemon = True
        pub_thread.start()

    def __in_thread(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("", self.port))

        while True:
            data, addr = sock.recvfrom(1024)
            self.__update(data.decode())

    def __pub_thread(self):
        starttime = time.time()
        frequency = 20
        period = 0.05
        while True:
            #print("=================")
            #print(time.time()-starttime)
            time.sleep(period-((time.time() - starttime) % period))
            self.pub.publish(self.msg)

    def __update(self, data):
        #print(data)
        objs = json.loads(data)
        cid = objs[u'cid']
        
        for obj in objs[u'objs']:
            oid = obj[u'oid']
            self.msg.objs[oid].cams[cid-1].pos = obj[u'pos']
            self.msg.objs[oid].cams[cid-1].update_time = rospy.get_rostime()
            print(self.msg)

        """
        for obj in objs[u'objs']:              
            self.indata[obj[u'oid']][cid] = {
                    'pos': obj[u'pos'],
                    'update_time': time.time()
                    }
        #print("in_id: " + str(self.in_id))
        #print("new message " + str(self.in_id) + ": ")
        #print(self.indata)
        print(json.dumps(self.indata, sort_keys=True, indent=2))
        """




def main():
    rospy.init_node("ncscon_node")
    thread1 = NcsconThread(5566, "ncscon_topic")

    rospy.spin()



if __name__ == "__main__":
    main()


