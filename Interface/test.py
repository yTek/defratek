from time import sleep
import ttk
from Tkinter import *
from PIL import ImageTk, Image
import os

class term(Frame):
	def __init__(self, master, size, color):
		Frame.__init__(self, master, height=150, width=1500)
		self.wid = self.winfo_id()
		os.system('xterm -into %d -geometry %dx%d -fg %s -sb &' % (self.wid, size[0], size[1], color))


class cameratabmanager():
	def __init__(self,master,dronelist):
		self.master=master
		self.labellist={}
		self.nb = ttk.Notebook(master)
		for drone in dronelist:
			tab = ttk.Frame(self.nb)

			self.nb.add(tab, text=drone['name'])
			self.labellist[drone['name']] = cameradisplay(tab, drone['image'], 0.7)

		self.nb.grid(row=0,column=0,sticky='wn')

class cameradisplay(Label):
	def __init__(self,master,img,scale):


		self.master = master
		self.image = ImageTk.PhotoImage(img.resize((int(img.size[0]*scale),int(img.size[1]*scale)), Image.ANTIALIAS))
		
		Label.__init__(self, master, image=self.image)
		self.pack()

class mastergui:
	def __init__(self, master,dronelist):
		self.master = master
		master.title("A simple GUI")

		self.tabs = cameratabmanager(master,dronelist)
		
		self.waypoints = Label(master, text="  WAYPOINTS MANAGER  ",bg='#0000FF')
		self.waypoints.grid(row=0,column=1,sticky='wens')

		self.terminal = term(master,(246,15), "white")
		self.terminal.grid(row=1,column=0,columnspan=2,sticky='wens')
		
		self.terminal2 = term(master,(246,15),"grey")
		self.terminal2.grid(row=2,column=0,columnspan=2,sticky='wens')
		
		self.terminal3 = term(master,(246,15),"white")
		self.terminal3.grid(row=3,column=0,columnspan=2,sticky='wens')

if __name__ == "__main__":

	root = Tk()
	dronelist=[
	{'name':'drone1','image': Image.open('mockimg.jpg')},
	{'name':'drone2','image': Image.open('mockimg2.jpg')},
	{'name':'drone3','image': Image.open('mockimg2.jpg')},
	]

	gui = mastergui(root,dronelist)

	root.grid_columnconfigure(index=1, weight=2)
	root.grid_rowconfigure(index=1, weight=1)

	while True:
		root.update_idletasks()
		root.update()

		#dronelist update by external process
		dronelist=[
		{'name':'drone1','image': Image.open('mockimg.jpg')},
		{'name':'drone2','image': Image.open('mockimg2.jpg')},
		{'name':'drone3','image': Image.open('mockimg2.jpg')},
		]

		#Tabs hot reload
		if not len(dronelist) is len(gui.tabs.labellist):
			print('Faire un hotreload des tabs ici')

		#Camera hot reload
		imagetemp={}
		for drone in dronelist:
			if drone['name'] == gui.tabs.nb.tab(gui.tabs.nb.select(), "text"):
				imagetemp[drone['name']] = ImageTk.PhotoImage(drone['image'].resize((int(drone['image'].size[0]*0.7),int(drone['image'].size[1]*0.7)), Image.ANTIALIAS))
				gui.tabs.labellist[drone['name']].configure(image=imagetemp[drone['name']])
