import tkinter as tk
from tkinter import ttk
from tkinter.ttk import *
import subprocess
import json

def launch(arg: int, target_list: list = []):
    match arg:
        case 0:
            subprocess.run("./run_target_tracking.sh")
            print("Running target tracking")
        case 1:
            subprocess.run("./run_navigation.sh")
            print("Running nav")
        case 2:
            subprocess.run("./run_test.sh")
            print("Test")
        case 3:
            # just make this case update the text file that you have or change the node with the input "target_list"
            fd = open("targets.txt", 'w+')
            for i in target_list:
                fd.write(str(i) + "\n")

            print("Updated list", target_list, type(target_list))

def main():
    window = tk.Tk()
    window.title("SDP Team 12 Gui")
    window.geometry("600x400+10+20")  # Set window size and position
    # ttk.Style().configure("TButton", padding=6, relief="flat",
    #    background="#ccc")

    btn1 = ttk.Button(text="Launch Target Tracking", command=lambda:launch(0))
    btn1.pack()
    btn2 = ttk.Button(text="Launch Nav", command=lambda:launch(1))
    btn2.pack()
    btn3 = ttk.Button(text="Launch Test", command=lambda:launch(2))
    btn3.pack()
    inputL = ttk.Entry(textvariable="enter list of targets",width=50)
    inputL.pack()
    inputL.insert(0,string="Enter list of targets like this => [1,59,2]")
    btn3 = ttk.Button(text="Update Targets", command=lambda:launch(3,json.loads(inputL.get())))
    btn3.pack()

    # Add widgets here (e.g., labels, buttons, etc.)

    # Start the GUI event loop
    # subprocess.run("./system_init.sh")
    window.mainloop()

if __name__ == "__main__":
    main()