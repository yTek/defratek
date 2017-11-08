from time import sleep
import ttk
from Tkinter import *
from PIL import ImageTk, Image
import os

class simframe(Frame):
	def __init__(self,master):
		self.master=master

		Frame.__init__(self, master)

		self.startstopbutton=Button(self, text="Start Simulation")
		self.takeoffbutton=Button(self, text="Takeoff")
		self.landbutton=Button(self, text="Land")

		self.startstopbutton.pack(side=LEFT, pady=4, fill=BOTH)
		self.takeoffbutton.pack(side=LEFT, pady=4, fill=BOTH)
		self.landbutton.pack(side=LEFT, pady=4, fill=BOTH)

		
class waypointinput(Frame):
	def __init__(self,master,waypointlist):
		self.master=master
		self.wpl=waypointlist
		Frame.__init__(self, master)

		self.xinput=Entry(self, width=6)
		self.yinput=Entry(self, width=6)
		self.zinput=Entry(self, width=6)
		self.minusbutton=Button(self, text="X", command=self.deletewaypoint)
		self.upbutton=self.downbutton=Button(self, text="^",  command=self.movewaypointup)
		self.downbutton=Button(self,text="v", command=self.movewaypointdown)

		self.xinput.pack(side=LEFT, padx=4)
		self.yinput.pack(side=LEFT, padx=4)
		self.zinput.pack(side=LEFT, padx=4)
		self.minusbutton.pack(side=LEFT, padx=2)
		self.upbutton.pack(side=LEFT)
		self.downbutton.pack(side=LEFT)
		
		waypointlist.append(self)
		self.pack()

	def deletewaypoint(self):
		self.pack_forget()
		self.wpl.pop(self.wpl.index(self))
		self.destroy()

	def movewaypointup(self):
		self.wpl.insert(self.wpl.index(self)-1, self.wpl.pop(self.wpl.index(self)))
		for i in self.wpl:
			i.pack_forget()
			i.pack()

	def movewaypointdown(self):
		self.wpl.insert(self.wpl.index(self)+1, self.wpl.pop(self.wpl.index(self)))
		for i in self.wpl:
			i.pack_forget()
			i.pack()

class waypointsmanager(Frame):
	def __init__(self,master):
		self.master=master
		
		Frame.__init__(self, master, borderwidth=2, relief=GROOVE)
		
		scrollbar = Scrollbar(self)
		scrollbar.pack(side=RIGHT, fill=Y)

		self.simframe=simframe(self)
		self.simframe.pack()

		self.waypointframe = Frame(self)
		#self.waypointframe = Frame(self, yscrollcommand=scrollbar.set)
		self.waypointlist=[]

		self.addbutton =Button(self, text="Add Waypoint",  command=self.addwaypoint)
		self.addbutton.pack()
		self.waypointframe.pack(side=RIGHT, fill=BOTH)

		#scrollbar.config(command=waypointframe.yview)
		self.grid(row=0,column=1,sticky='wens')

	def addwaypoint(self):
		waypointinput(self.waypointframe, self.waypointlist)

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
		master.title("THE BEST")
		
		
		self.tabs = cameratabmanager(master,dronelist)
		
		#waypointsbim = Frame(master)
		self.waypoint = waypointsmanager(master)
		
		
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
	{'name':'drone3','image': Image.open('mockimg3.jpg')},
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
		{'name':'drone3','image': Image.open('mockimg3.jpg')},
		]

		#Tabs hot reload
		if not len(dronelist) is len(gui.tabs.labellist):
			print('Faire un hotreload des tabs ici')

		"""
		#Camera hot reload
		imagetemp={}
		for drone in dronelist:
			print(drone['name'] == gui.tabs.nb.tab(gui.tabs.nb.select(), "text"))
			if drone['name'] == gui.tabs.nb.tab(gui.tabs.nb.select(), "text"):
				imagetemp[drone['name']] = ImageTk.PhotoImage(drone['image'].resize((int(drone['image'].size[0]*0.7),int(drone['image'].size[1]*0.7)), Image.ANTIALIAS))
				gui.tabs.labellist[drone['name']].configure(image=imagetemp[drone['name']])
		"""
