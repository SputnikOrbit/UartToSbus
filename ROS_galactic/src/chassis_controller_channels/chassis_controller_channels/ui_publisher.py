import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tkinter as tk

class GUIPublisher(Node):

    def __init__(self):
        super().__init__('gui_publisher')
        self.publisher_ = self.create_publisher(String, 'chassis_command', 10)
        self.init_ui()

    def init_ui(self):
        self.root = tk.Tk()
        self.root.title("Chassis Controller")

        self.channels = [tk.IntVar() for _ in range(6)]

        for i in range(6):
            tk.Label(self.root, text=f"Channel {i+1}").grid(row=i, column=0)
            tk.Scale(self.root, from_=0, to=2000, orient=tk.HORIZONTAL, variable=self.channels[i]).grid(row=i, column=1)
            tk.Button(self.root, text="Trim", command=lambda i=i: self.trim_channel(i)).grid(row=i, column=2)
        self.send_command()  # Start the periodic sending
        self.root.mainloop()

    def trim_channel(self, index):
        self.channels[index].set(1000)

    def send_command(self):
        command = f"ch1{self.channels[0].get()}ch2{self.channels[1].get()}ch3{self.channels[2].get()}ch4{self.channels[3].get()}ch5{self.channels[4].get()}ch6{self.channels[5].get()}"
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.root.after(50, self.send_command)  # Schedule the next call to this function (20Hz)

def main(args=None):
    rclpy.init(args=args)
    gui_publisher = GUIPublisher()
    rclpy.spin(gui_publisher)
    gui_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
