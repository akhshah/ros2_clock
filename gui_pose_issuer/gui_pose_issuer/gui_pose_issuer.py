import sys
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from python_qt_binding.QtWidgets import QApplication, QWidget
from python_qt_binding.QtGui import QPainter, QPen, QColor
from python_qt_binding.QtCore import Qt, QPoint, QTimer

from geometry_msgs.msg import PoseStamped

class GuiPoseIssuerWidget(QWidget):
    def __init__(self, node, radius):
        super(GuiPoseIssuerWidget, self).__init__()

        self.setMinimumSize(400, 400)
        self.setWindowTitle("Clock Pose Issuer")

        self.radius = radius
        self.node = node

        self.click_point = None
        self.last_pose = None

        self.publisher = node.create_publisher(PoseStamped, 'gui_pose', 10)
        timer_period = 0.5 # Units: seconds
        self.cleanup_timer = self.node.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # There is not published pose so nothing to clear.
        if not self.last_pose:
            return

        last_timestamp = Time.from_msg(self.last_pose.header.stamp)

        seconds_since_publish = (self.node.get_clock().now() -
                                 last_timestamp).nanoseconds / 1e9
        if seconds_since_publish >= 30:
            self.click_point = None
            self.last_pose = None

            self.node.get_logger().info('Publishing empty pose')
            self.publisher.publish(PoseStamped())
            self.update()

    def getClockRadius(self):
        return min(self.width(), self.height()) / 2 - 20

    def paintEvent(self, event):
        painter = QPainter(self)

        painter.setRenderHint(QPainter.Antialiasing)

        # Radius of the clock face is slightly smaller than window, so it is
        # visible.
        radius = self.getClockRadius()
        center = QPoint(self.width() // 2, self.height() // 2)

       # Draw clock face circle
        painter.setPen(QPen(Qt.black, 3))
        painter.drawEllipse(center, int(radius), int(radius))

        # Draw hour markers
        for hour in range(12):
            angle = math.radians(hour * 30)
            x1 = center.x() + (radius - 10) * math.cos(angle)
            y1 = center.y() + (radius - 10) * math.sin(angle)
            x2 = center.x() + (radius) * math.cos(angle)
            y2 = center.y() + (radius) * math.sin(angle)

            painter.drawLine(QPoint(int(x1), int(y1)), QPoint(int(x2), int(y2)))

        # Draw point clicked
        painter.setPen(QPen(Qt.black, 8))
        if self.click_point:
            painter.drawPoint(self.click_point)


    def mousePressEvent(self, event):
        pos = event.pos()
        self.click_point = pos
        self.update()

        center = QPoint(self.width() // 2, self.height() // 2)
        dx = pos.x() - center.x()

        # NOTE(akhil): Qt frames have the top left most point at 0, 0 and so we
        # have to flip the sign here to get the correct pose.
        dy = center.y() - pos.y()

        angle = math.atan2(dy, dx)

        x = self.radius * math.cos(angle)
        y = self.radius * math.sin(angle)

        yaw = math.atan2(-dx, dy)

        msg = PoseStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'inertial'
        msg.pose.position.x = x
        msg.pose.position.y = y

        msg.pose.orientation.x = 0.
        msg.pose.orientation.y = 0.
        msg.pose.orientation.z = math.sin(0.5 * yaw)
        msg.pose.orientation.w = math.cos(0.5 * yaw)

        self.last_pose = msg

        self.node.get_logger().info(f'Mouse clicked. Pose = ({x:.2f}, {y:.2f}, {yaw:.2f})')
        self.publisher.publish(msg)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Space:
            msg = PoseStamped()

            self.node.get_logger().info('Space pressed, publishing empty pose')
            self.publisher.publish(PoseStamped())

            self.click_point = None
            self.last_pose = None
            self.update()

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)

    node = rclpy.create_node('gui_pose_issuer')
    widget = GuiPoseIssuerWidget(node, 1.)
    widget.show()

    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.01))
    timer.start(50) # Units: 20 Hz

    exit_code = app.exec_()

    timer.stop()
    node.destroy_node()
    rclpy.shutdown()

    sys.exit(exit_code)

if __name__  == '__main__':
    main()
