import tkinter as tk

# Create GUI object
app = tk.Tk()

def submit_changes():
    print(Submitted)
    
# Button 1
btn1_text = tk.StringVar()
btn1_label = tk.Label(app, text='Button Name 1', font=('bold', 14), pady=15)
btn1_label.grid(row=0, column=0, sticky=tk.W)
btn1_entry = tk.Entry(app, textvariable=btn1_text)
btn1_entry.grid(row=0, column=1)

# Button 2
btn2_text = tk.StringVar()
btn2_label = tk.Label(app, text='Button Name 2', font=('bold', 14))
btn2_label.grid(row=0, column=2, sticky=tk.W)
btn2_entry = tk.Entry(app, textvariable=btn2_text)
btn2_entry.grid(row=0, column=3)

# Button 3
btn3_text = tk.StringVar()
btn3_label = tk.Label(app, text='Button Name 3', font=('bold', 14), pady=15)
btn3_label.grid(row=1, column=0, sticky=tk.W)
btn3_entry = tk.Entry(app, textvariable=btn3_text)
btn3_entry.grid(row=1, column=1)

# Button 4
btn4_text = tk.StringVar()
btn4_label = tk.Label(app, text='Button Name 4', font=('bold', 14))
btn4_label.grid(row=1, column=2, sticky=tk.W)
btn4_entry = tk.Entry(app, textvariable=btn4_text)
btn4_entry.grid(row=1, column=3)

# Button 5
btn5_text = tk.StringVar()
btn5_label = tk.Label(app, text='Button Name 5', font=('bold', 14), pady=15)
btn5_label.grid(row=2, column=0, sticky=tk.W)
btn5_entry = tk.Entry(app, textvariable=btn5_text)
btn5_entry.grid(row=2, column=1)

# Button 6
btn6_text = tk.StringVar()
btn6_label = tk.Label(app, text='Button Name 6', font=('bold', 14))
btn6_label.grid(row=2, column=2, sticky=tk.W)
btn6_entry = tk.Entry(app, textvariable=btn6_text)
btn6_entry.grid(row=2, column=3)

# UI Buttons
submit_btn = tk.Button(app, text='Submit Changes', width=12, command=submit_changes)
submit_btn.grid(row=3, column=0, pady=20)

app.title('Video Editor Configuration Utility')
app.geometry('1000x600')

# Start the utility
app.mainloop()
