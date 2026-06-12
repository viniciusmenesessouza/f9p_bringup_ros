#!/usr/bin/env python3
import os
import signal
import subprocess
from datetime import datetime

import rospy


class RosbagRecorder:
    def __init__(self):
        self.bag_dir = rospy.get_param("~bag_dir", os.path.join(os.path.expanduser("~"), "bags"))
        self.tag = rospy.get_param("~tag", "VIENA")
        self.use_lz4 = bool(rospy.get_param("~use_lz4", True))
        self.tcpnodelay = bool(rospy.get_param("~tcpnodelay", True))
        self.buffsize_mb = int(rospy.get_param("~buffsize_mb", 1024))
        self.record_tf = bool(rospy.get_param("~record_tf", False))
        self.topics = rospy.get_param("~topics", [])

        if not isinstance(self.topics, list) or len(self.topics) == 0:
            rospy.logfatal("~topics must be a non-empty YAML list")
            raise RuntimeError("No topics provided")

        # Ensure bag dir exists
        os.makedirs(self.bag_dir, exist_ok=True)

        # Filename: HHMMSS_DDMMYY_VIENA.bag
        stamp = datetime.now().strftime("%H%M%S_%d%m%y")
        self.bag_file = os.path.join(self.bag_dir, f"{stamp}_{self.tag}.bag")

        # Build rosbag command
        cmd = ["rosbag", "record", "-O", self.bag_file]

        if self.use_lz4:
            cmd.append("--lz4")
        if self.tcpnodelay:
            cmd.append("--tcpnodelay")

        cmd.append(f"--buffsize={self.buffsize_mb}")

        if self.record_tf:
            self.topics = self.topics + ["/tf", "/tf_static"]

        cmd += self.topics

        rospy.loginfo("Recording rosbag to: %s", self.bag_file)
        rospy.loginfo("Topics (%d): %s", len(self.topics), " ".join(self.topics))

        # Start rosbag in its own process group so we can stop it cleanly
        self.proc = subprocess.Popen(cmd, preexec_fn=os.setsid)

        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        if getattr(self, "proc", None) is None:
            return
        if self.proc.poll() is not None:
            return

        rospy.loginfo("Stopping rosbag...")
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
        except Exception:
            pass

        try:
            self.proc.wait(timeout=10)
        except Exception:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
            except Exception:
                pass


if __name__ == "__main__":
    rospy.init_node("record_rosbag")
    RosbagRecorder()
    rospy.spin()

