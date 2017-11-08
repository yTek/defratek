from Tkinter import *
import ttk

if __name__ == "__main__":
	root = Tk()

	scrollbar = Scrollbar(root)
	scrollbar.pack(side=RIGHT, fill=Y)

	listbox = Listbox(root, yscrollcommand=scrollbar.set)
	for i in range(10):
		listbox.insert(END, str(i))
	listbox.pack(side=LEFT, fill=BOTH)

	waypointframe = Frame(root)

	xinput=Entry(waypointframe,text="X")
	yinput=Entry(waypointframe,text="Y")
	zinput=Entry(waypointframe,text="Z")

	xinput.pack()
	yinput.pack()
	zinput.pack()
	
	waypointframe.pack()

	root.mainloop()