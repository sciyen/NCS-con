#!/usr/bin/env python2.7
import time
import json
import rospy
import socket
import threading

PUB_MODE = '2d'

if PUB_MODE == '2d':
    from ncs_con.msg import Con_msg_2d
    from ncs_con.msg import cam_msg_2d
    from ncs_con.msg import obj_msg_2d

if PUB_MODE == '3d':
    from ncs_con.msg import Con_msg
    from ncs_con.msg import cam_msg
    from ncs_con.msg import obj_msg


class NcsconThread():
    def __init__(self, port, pubTopic):
        self.indata = [{} for i in range(5)]
        self.port = port

        # Constructing message
        if PUB_MODE == '2d':
            self.msg = Con_msg_2d()
            self.pub = rospy.Publisher(pubTopic, Con_msg_2d, queue_size=1)
        if PUB_MODE == '3d':
            self.msg = Con_msg()
            self.pub = rospy.Publisher(pubTopic, Con_msg, queue_size=1)

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
        frequency = 10
        period = 0.1
        while True:
            # print("=================")
            # print(time.time()-starttime)
            time.sleep(period-((time.time() - starttime) % period))
            self.pub.publish(self.msg)
            self.__reset_buffer()

    def __reset_buffer(self):
        if PUB_MODE == '2d':
            self.msg = Con_msg_2d()
        if PUB_MODE == '3d':
            self.msg = Con_msg()

    def __update(self, data):
        # New incoming socket data from pi
        objs = json.loads(data)
        cid = objs[u'cid']

        if PUB_MODE == '2d':
            cm = cam_msg_2d()
            om = obj_msg_2d()
        if PUB_MODE == '3d':
            cm = cam_msg()
            om = obj_msg()

        cm.cid = cid

        obj_list = {}
        for obj in objs[u'objs']:
            oid = obj[u'oid']
            if oid in obj_list:
                continue

            obj_list[oid] = 1
            if PUB_MODE == '2d':
                om = obj_msg_2d()
            if PUB_MODE == '3d':
                om = obj_msg()
            om.cid = cid
            om.oid = oid
            om.pos = obj[u'pos']
            om.update_time = rospy.get_rostime()
            cm.objs.append(om)

        # check if cid exist
        try:
            cid_list = [c.cid for c in self.msg.cams]
            idx = cid_list.index(cid)
            self.msg.cams[idx] = cm
        except ValueError:
            # cid not exist
            self.msg.cams.append(cm)

        #print(json.dumps(data, sort_keys=True, indent=2))


def main():
    rospy.init_node("ncscon_node")
    thread1 = NcsconThread(5566, "ncscon_topic")
    print("Start listening")
    rospy.spin()


if __name__ == "__main__":
    main()
