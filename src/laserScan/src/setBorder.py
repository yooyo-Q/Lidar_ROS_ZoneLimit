# -*- coding: utf-8 -*-
import Tkinter as tk
import tkMessageBox as messagebox
import json
import os

class BorderConfigApp:
    def __init__(self, master):
        self.master = master
        master.title("Border Configuration")
        master.geometry("300x200")

        self.config_file = "border.json"
        self.config = self.load_config()

        # 左边界
        tk.Label(master, text="左边界:").grid(row=0, column=0, padx=10, pady=5)
        self.left_entry = tk.Entry(master)
        self.left_entry.grid(row=0, column=1, padx=10, pady=5)
        self.left_entry.insert(0, str(self.config.get('left_Border', -2)))

        # 右边界
        tk.Label(master, text="右边界:").grid(row=1, column=0, padx=10, pady=5)
        self.right_entry = tk.Entry(master)
        self.right_entry.grid(row=1, column=1, padx=10, pady=5)
        self.right_entry.insert(0, str(self.config.get('right_Border', 3)))

        # 前边界
        tk.Label(master, text="前边界:").grid(row=2, column=0, padx=10, pady=5)
        self.front_entry = tk.Entry(master)
        self.front_entry.grid(row=2, column=1, padx=10, pady=5)
        self.front_entry.insert(0, str(self.config.get('front_Border', 20)))

        # 保存按钮
        save_button = tk.Button(master, text="保存", command=self.save_config)
        save_button.grid(row=3, column=0, columnspan=2, pady=20)

    def load_config(self):
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, 'r') as file:
                    return json.load(file)
            except json.JSONDecodeError:
                messagebox.showwarning("警告", "配置文件格式错误，将使用默认值")
        else:
            messagebox.showinfo("提示", "配置文件不存在，将创建新文件")
        
        # 返回默认配置
        return {
            'left_Border': -2,
            'right_Border': 3,
            'front_Border': 20
        }

    def save_config(self):
        try:
            new_config = {
                'left_Border': float(self.left_entry.get()),
                'right_Border': float(self.right_entry.get()),
                'front_Border': float(self.front_entry.get())
            }
            with open(self.config_file, 'w') as file:
                json.dump(new_config, file, indent=2)
            messagebox.showinfo("成功", "配置已保存")
        except ValueError:
            messagebox.showerror("错误", "请输入有效的数字")

if __name__ == "__main__":
    root = tk.Tk()
    app = BorderConfigApp(root)
    root.mainloop()