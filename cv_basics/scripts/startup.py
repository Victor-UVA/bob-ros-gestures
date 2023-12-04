import tkinter as tk
from tkinter import messagebox
import subprocess
import os

class ROSLauncher:
    def __init__(self, master):
        self.master = master
        master.title("ROS Launcher")

        # Create Launch ROS button
        self.launch_button = tk.Button(master, text="Launch ROS", fg="white", bg="green", command=self.launch_ros, width=15, height=10)
        self.launch_button.pack(padx=500, pady=150)

        # Create Kill ROS button
        self.kill_button = tk.Button(master, text="Kill ROS", fg="white", bg="red", command=self.kill_ros, width=15, height=10)
        self.kill_button.pack(padx=500, pady=150)

    def launch_ros(self):
        try:
            # Change directory to ~/ros/seed_ws
            os.chdir(os.path.expanduser('~/ros/seed_ws'))

            # Open a new terminal window and run the commands
            subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', 'source devel/setup.bash && roslaunch seed_r7_bringup moveit.launch robot_model:=typeg'])

        except Exception as e:
            messagebox.showerror("Error", f"An error occurred: {str(e)}")

    def kill_ros(self):
        try:
            # Kill ROS processes using pkill
            subprocess.run(['pkill', '-f', 'roslaunch'])

        except Exception as e:
            messagebox.showerror("Error", f"An error occurred: {str(e)}")

if __name__ == "__main__":
    root = tk.Tk()
    app = ROSLauncher(root)
    root.mainloop()
