# https://gist.github.com/pryre/8d6f44e18d52efb616f64403de0838ef
# sudo apt install ros-noetic-cv-bridge

import sys
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class ROSImage2Video:
    def __init__(self) -> None:
        video_dir = sys.argv[1]
        video_name = sys.argv[2]
        self.video_path = "{}/{}.mp4".format(video_dir, video_name)

        self.img_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)

        self.is_inited = False
        self.flag = False

        self.vw = None
        self.count = 0

    def callback(self, data):
        br = CvBridge()

        frame = br.imgmsg_to_cv2(data, 'bgr8')

        print(frame.shape)

        if not self.is_inited:
            fps = 20
            height, width, _ = frame.shape
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            if self.vw is None:
                self.vw = cv2.VideoWriter(self.video_path, fourcc, fps, (width, height))
            self.is_inited = True
        else:
            if False:
                cv2.imshow("camera", frame)
                k = cv2.waitKey(5)
                if self.flag:
                    self.vw.write(frame)
                    print("recording frame count {}", format(self.count))
                    self.count += 1
                if k == ord('s'):
                    self.flag = True
                    print("start recording video {}".format(self.video_path))
                elif k & 0xFF == ord('q') or k == 27:
                    print("done")
                    self.shutdown()
            else:
                self.vw.write(frame)
                print("recording frame count {}", format(self.count))
                self.count += 1

    def shutdown(self):
        if self.img_sub is not None:
            self.img_sub.unregister()
        self.vw.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    rospy.init_node("video_sub_py", anonymous=True)

    iv = ROSImage2Video()
    rospy.on_shutdown(iv.shutdown)

    rospy.spin()
