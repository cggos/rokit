import sys
import rosbag
from PIL import Image
import os
import numpy as np
import cv2

sys.path.append('/opt/ros/melodic/lib/python2.7/dist-packages')

idx_start = 0
num_need = 100

if __name__ == '__main__':
    bag_dir = sys.argv[1]
    bag_name = sys.argv[2]
    img_topic = sys.argv[3]  # '/camera/color/image_raw'
    img_channel = 3
    out_dir = 'imgs_out'
    out_root = bag_dir
    # out_root = '/home/cg/projects/dataset/rs_cam/t265/nj_lawn/'
    img_dir = os.path.join(bag_dir, out_dir)
    if not os.path.exists(img_dir):
        os.makedirs(img_dir)
    bag_file = os.path.join(bag_dir, bag_name)
    bag = rosbag.Bag(bag_file)
    conx = bag._get_connections(topics=img_topic)
    indices = bag._get_indexes(conx)
    try:
        index = next(indices)
    except:
        raise RuntimeError("Could not find topic {0} in {1}.".format(img_topic, bag))
    num_max = len(index)
    num_dur = num_max / num_need
    idx = 0
    idx_out = idx_start
    imgname = os.path.join(img_dir, '{:0>5d}.png')
    for topic, msg, t in bag.read_messages(topics=img_topic):
        if idx % num_dur == 0:
            header = msg.header
            header_seq = header.seq
            stamp_sec = header.stamp.secs
            stamp_nsec = header.stamp.nsecs
            data = msg.data  # bytes
            img = np.frombuffer(data, dtype=np.uint8)
            img = img.reshape(msg.height, msg.width, img_channel)
            if img_channel == 3:
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            cv2.imwrite(imgname.format(idx_out), img)
            print(
                '{:0>5d} {:0>5d} {} {} {}'.format(
                    idx_out, idx, header_seq, stamp_sec, stamp_nsec
                )
            )
            idx_out += 1
        idx += 1
