import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

# 按鍵對應控制表
move_bindings = {
    'i': (1, 0, 0, 0),
    'o': (1, 0, 0, -1),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    'u': (1, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1),
    't': (0, 0, 1, 0),
    'b': (0, 0, -1, 0),
    'h': (0, 0.5, 0, 0),  # 新增按鍵 h，控制 linear.y
}

speed_bindings = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
    'w': (1.1, 1.0),
    'x': (0.9, 1.0),
    'e': (1.0, 1.1),
    'c': (1.0, 0.9),
}

class TeleopTwistKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_twist_keyboard')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel_key', 10)
        # self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.speed = 0.5
        self.turn = 1.0
        self.print_instructions()
    
    def print_instructions(self):
        print("""
        控制指令:
           u    i    o
           j    k    l
           m    ,    .

        上下移動 (Z 軸):
        t : up (+z)
        b : down (-z)

        左右移動 (Y 軸):
        h : linear.y = 0.5

        調整速度:
        q/z : 調整速度 +/- 10%
        w/x : 線性速度 +/- 10%
        e/c : 旋轉速度 +/- 10%

        按下 CTRL+C 離開
        """)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run(self):
        try:
            while True:
                key = self.get_key()
                if key in move_bindings.keys():
                    x, y, z, th = move_bindings[key]
                elif key in speed_bindings.keys():
                    speed_factor, turn_factor = speed_bindings[key]
                    self.speed *= speed_factor
                    self.turn *= turn_factor
                    print(f"目前速度: speed {self.speed:.2f}, turn {self.turn:.2f}")
                    continue
                else:
                    x, y, z, th = 0, 0, 0, 0
                    if key == '\x03':  # CTRL+C 結束
                        break

                twist = Twist()
                twist.linear.x = float(x * self.speed)
                twist.linear.y = float(y)  # 確保 y 為浮點數
                twist.linear.z = float(z * self.speed)
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = float(th * self.turn)
                self.publisher_.publish(twist)

        except Exception as e:
            print(e)
        finally:
            twist = Twist()
            self.publisher_.publish(twist)

def main(args=None):
    global settings
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init(args=args)
    node = TeleopTwistKeyboard()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
