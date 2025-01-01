import tkinter as tk
import vgamepad as vg
import threading
import time

class VirtualGamepadGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Virtual Gamepad Controller")
        
        # Create virtual gamepad
        self.gamepad = vg.VX360Gamepad()
        
        # Thread for continuous updates
        self.update_thread = None
        self.is_updating = False
        
        # Create GUI components
        self.create_layout()
        
    def create_layout(self):
        # Main frame
        main_frame = tk.Frame(self.root, padx=10, pady=10)
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Buttons Section
        buttons_frame = tk.LabelFrame(main_frame, text="Buttons")
        buttons_frame.pack(fill=tk.X, pady=5)
        
        # Button mapping
        button_mapping = [
            ('A', vg.XUSB_BUTTON.XUSB_GAMEPAD_A),
            ('B', vg.XUSB_BUTTON.XUSB_GAMEPAD_B),
            ('X', vg.XUSB_BUTTON.XUSB_GAMEPAD_X),
            ('Y', vg.XUSB_BUTTON.XUSB_GAMEPAD_Y),
            ('Left Shoulder', vg.XUSB_BUTTON.XUSB_GAMEPAD_LEFT_SHOULDER),
            ('Right Shoulder', vg.XUSB_BUTTON.XUSB_GAMEPAD_RIGHT_SHOULDER),
            ('Back', vg.XUSB_BUTTON.XUSB_GAMEPAD_BACK),
            ('Start', vg.XUSB_BUTTON.XUSB_GAMEPAD_START),
            ('Left Thumb', vg.XUSB_BUTTON.XUSB_GAMEPAD_LEFT_THUMB),
            ('Right Thumb', vg.XUSB_BUTTON.XUSB_GAMEPAD_RIGHT_THUMB)
        ]
        
        # Buttons Checkboxes
        self.button_vars = {}
        for i, (name, button) in enumerate(button_mapping):
            var = tk.BooleanVar()
            self.button_vars[button] = var
            cb = tk.Checkbutton(buttons_frame, text=name, variable=var, 
                                command=lambda b=button: self.toggle_button(b))
            cb.grid(row=i//3, column=i%3, sticky='w')
        
        # D-Pad Section
        dpad_frame = tk.LabelFrame(main_frame, text="D-Pad")
        dpad_frame.pack(fill=tk.X, pady=5)
        
        dpad_buttons = [
            ('Up', vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_UP),
            ('Down', vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_DOWN),
            ('Left', vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_LEFT),
            ('Right', vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_RIGHT)
        ]
        
        for i, (name, button) in enumerate(dpad_buttons):
            var = tk.BooleanVar()
            self.button_vars[button] = var
            cb = tk.Checkbutton(dpad_frame, text=name, variable=var, 
                                command=lambda b=button: self.toggle_button(b))
            cb.grid(row=i//2, column=i%2, sticky='w')
        
        # Joysticks Section
        joysticks_frame = tk.LabelFrame(main_frame, text="Joysticks")
        joysticks_frame.pack(fill=tk.X, pady=5)
        
        # Left Joystick
        tk.Label(joysticks_frame, text="Left Joystick").pack()
        self.left_x = tk.Scale(joysticks_frame, from_=-1, to=1, resolution=0.1, 
                               orient=tk.HORIZONTAL, label='X')
        self.left_x.set(0)
        self.left_x.pack()
        
        self.left_y = tk.Scale(joysticks_frame, from_=-1, to=1, resolution=0.1, 
                               orient=tk.HORIZONTAL, label='Y')
        self.left_y.set(0)
        self.left_y.pack()
        
        # Right Joystick
        tk.Label(joysticks_frame, text="Right Joystick").pack()
        self.right_x = tk.Scale(joysticks_frame, from_=-1, to=1, resolution=0.1, 
                                orient=tk.HORIZONTAL, label='X')
        self.right_x.set(0)
        self.right_x.pack()
        
        self.right_y = tk.Scale(joysticks_frame, from_=-1, to=1, resolution=0.1, 
                                orient=tk.HORIZONTAL, label='Y')
        self.right_y.set(0)
        self.right_y.pack()
        
        # Triggers Section
        triggers_frame = tk.LabelFrame(main_frame, text="Triggers")
        triggers_frame.pack(fill=tk.X, pady=5)
        
        self.left_trigger = tk.Scale(triggers_frame, from_=0, to=1, resolution=0.1, 
                                     orient=tk.HORIZONTAL, label='Left Trigger')
        self.left_trigger.set(0)
        self.left_trigger.pack()
        
        self.right_trigger = tk.Scale(triggers_frame, from_=0, to=1, resolution=0.1, 
                                      orient=tk.HORIZONTAL, label='Right Trigger')
        self.right_trigger.set(0)
        self.right_trigger.pack()
        
        # Update Controls
        control_frame = tk.Frame(main_frame)
        control_frame.pack(fill=tk.X, pady=5)
        
        self.update_btn = tk.Button(control_frame, text="Start Continuous Update", 
                                    command=self.toggle_continuous_update)
        self.update_btn.pack(side=tk.LEFT, expand=True)
        
        self.reset_btn = tk.Button(control_frame, text="Reset Gamepad", 
                                   command=self.reset_gamepad)
        self.reset_btn.pack(side=tk.LEFT, expand=True)
    
    def toggle_button(self, button):
        """Toggle specific button press"""
        if self.button_vars[button].get():
            self.gamepad.press_button(button=button)
        else:
            self.gamepad.release_button(button=button)
        self.gamepad.update()
    
    def update_gamepad(self):
        """Update gamepad state with current GUI values"""
        # Update left joystick
        self.gamepad.left_joystick_float(
            x_value_float=self.left_x.get(), 
            y_value_float=self.left_y.get()
        )
        
        # Update right joystick
        self.gamepad.right_joystick_float(
            x_value_float=self.right_x.get(), 
            y_value_float=self.right_y.get()
        )
        
        # Update triggers
        self.gamepad.left_trigger_float(value_float=self.left_trigger.get())
        self.gamepad.right_trigger_float(value_float=self.right_trigger.get())
        
        # Update button states
        for button, var in self.button_vars.items():
            if var.get():
                self.gamepad.press_button(button=button)
            else:
                self.gamepad.release_button(button=button)
        
        # Send update to gamepad
        self.gamepad.update()
    
    def continuous_update_loop(self):
        """Continuous update loop for gamepad"""
        while self.is_updating:
            self.update_gamepad()
            time.sleep(0.1)  # 10 Hz update rate
    
    def toggle_continuous_update(self):
        """Toggle continuous gamepad updates"""
        if not self.is_updating:
            # Start continuous update
            self.is_updating = True
            self.update_thread = threading.Thread(target=self.continuous_update_loop)
            self.update_thread.start()
            self.update_btn.config(text="Stop Continuous Update")
        else:
            # Stop continuous update
            self.is_updating = False
            if self.update_thread:
                self.update_thread.join()
            self.update_btn.config(text="Start Continuous Update")
    
    def reset_gamepad(self):
        """Reset gamepad to default state"""
        # Reset all GUI controls
        self.left_x.set(0)
        self.left_y.set(0)
        self.right_x.set(0)
        self.right_y.set(0)
        self.left_trigger.set(0)
        self.right_trigger.set(0)
        
        # Uncheck all buttons
        for var in self.button_vars.values():
            var.set(False)
        
        # Reset gamepad
        self.gamepad.reset()
        self.gamepad.update()

def main():
    root = tk.Tk()
    app = VirtualGamepadGUI(root)
    root.mainloop()

if __name__ == '__main__':
    main()
