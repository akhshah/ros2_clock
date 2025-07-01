import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtGui import QPainter, QPen, QColor
from python_qt_binding.QtCore import Qt, QPoint, QTimer

from geometry_msgs.msg import PoseStamped


class GuiPoseIssuer(Plugin):
    def __init__(self, context):
        super(GuiPoseIssuer, self).__init__(context)
        self.setObjectName('GuiPoseIssuer')

        rclpy.init()
        self.node = rclpy.create_node('gui_pose_issuer')

        self._widget = GuiPoseIssuerWidget(self.node, 1.)
        self._widget.setObjectName('GuiPoseIssuer')
        contex.add_widget(self._widget)

        self.setFocusPolicy(Qt.StrongFocus)

        # Timer to spin the node
        self.timer = QTimer()
        self.timer.timeout.connect(lambda: rclpy.spin_once(self.node,
                                                           timeout_sec = 0))
        self.timer.start(50) # 20 Hz

    def shutdown_plugin(self):
        self.node.destory_node()
        rclpy.shutdown()


class GuiPoseIssuerWidget(QWidget):
    def __init__(self, node, radius):
        super(GuiPoseIssuerWidget, self).__init__()

        self.setMinimumSize(400, 400)

        self.radius = radius
        self.node = node

        self.click_point = None
        self.last_pose = None

        self.publisher = node.create_publisher(PoseStamped, 'gui_pose', 10)
        timer_period = 0.5 # Units: seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

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

    def paintEvent(self, event):
        qp = QPainter(self)

        painter.setRenderHint(QPainter.Antialiasing)

        # Radius of the clock face is slightly smaller than window, so it is
        # visible.
        radius = min(self.width(), self.height()) / 2 - 20
        center = QPoint(self.width() // 2, self.height() // 2)

        # Draw clock face circle
        painter.setPen(Qpen(Qt.black, 3))
        painter.drawEllipse(center, int(radius), int(radius))

        # Draw hour markers
        for hour in range(12):
            angle = math.radians(hour * 30)
            x1 = center.x() + (radius - 10) * math.cos(angle)
            y1 = center.y() + (radius - 10) * math.sin(angle)
            x2 = center.x() + (radius) * math.cos(angle)
            y2 = center.y() + (radius) * math.sin(angle)

        painter.drawLine(Qpoint(x1, y1), QPoint(x2, y2))

        # Draw point clicked
        painter.SetPen(QPen(Qt.black, 8))
        if self.click_point:
            painter.drawPoint(self.click_point)


    def mousePressEvent(self, event):
        pos = event.pos()
        self.click_points = pos
        self.update()

        center = QPoint(self.width() // 2, self.height() // 2)
        dx = pos.x() - center.x()
        dy = pos.y() - center.y()

        angle = math.atan2(dy, dx)

        x = self.radius * math.cos(angle)
        y = self.radius * math.sin(angle)

        yaw = math.atan2(-dx, dy)

        msg = PoseStamped()
        msg.head.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'inertial'
        msg.pose.position.x = x
        msg.pose.position.y = y

        msg.pose.orientation.x = 0
        msg.pose.orientation.y = 0
        msg.pose.orientation.z = math.sin(0.5 * yaw)
        msg.pose.orientation.w = math.cos(0.5 * yaw)

        self.node.get_logger().info(f'Mouse clicked. Pose = ({x:.2f}, {y:.2f}, {yaw:.2f})')
        self.publisher.publish(msg)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Space:
            msg = PoseStamped()

            self.node.get_logger().info('Space pressed, publishing empty pose')

            self.click_point = None
            self.last_pose = None
            self.update()
