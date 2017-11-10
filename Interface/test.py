#-*- coding: utf-8 -*-

from time import sleep
import ttk
from Tkinter import *
from PIL import ImageTk, Image
import os

class simframe(Frame):
	def __init__(self, master, waypointlist):
		self.master=master
		self.wpl=waypointlist

		Frame.__init__(self, master)

		self.initbutton=Button(self, text="INIT", bg='#BFA903', fg='white', command=self.initialize)
		self.startstopbutton=Button(self, text="START", bg='green', fg='white', command=self.startsim)
		self.landall=Button(self, text="Land All", command=self.landall)
		self.emergencylandbutton=Button(self, text="Emergency Land All",fg='red', command=self.emergencyland)

		self.initbutton.pack(side=LEFT, pady=4, fill=BOTH)
		self.startstopbutton.pack(side=LEFT, pady=4, fill=BOTH)
		self.landall.pack(side=LEFT, pady=4, fill=BOTH)
		self.emergencylandbutton.pack(side=LEFT, pady=4, fill=BOTH)

	def initialize(self):
		pass
		#Init les launch
		self.iplist=''
		for drone in self.master.master.dronelist:
			self.iplist+=' '+drone['ip']

		print('../Launcher_multi_drone/launch_multidrone.sh'+self.iplist)
		#os.system('../Launcher_multi_drone/launch_multidrone.sh'+self.iplist)

	def startsim(self):	
		try:
			#Read all waypoints
			waypointfinal=[]
			
			for waypoint in self.wpl:
				x=float(waypoint.xinput.get())
				y=float(waypoint.yinput.get())
				z=float(waypoint.zinput.get())
				if(z<1):
					z=1.0
				waypointfinal.append((x,y,z))
			
			#Toggle to stopsim if no ValueError and if waypointfinal is not empty
			if waypointfinal:
				self.initbutton.config(state="disabled", bg='SystemButtonFace')
				self.master.addbutton.config(state="disabled")
				self.master.generatemapbutton.config(state="disabled")

				for waypoint in self.wpl:
					waypoint.xinput.config(state="disabled")
					waypoint.yinput.config(state="disabled")
					waypoint.zinput.config(state="disabled")
					waypoint.minusbutton.config(state="disabled")
					waypoint.upbutton.config(state="disabled")
					waypoint.downbutton.config(state="disabled")

				self.startstopbutton.config(text=' STOP ', bg='red', activebackground='#FF6666', command=self.stopsim)

				print(waypointfinal)
			else:
				raise ValueError

		except ValueError:
			print('Inputs are invalid or course is empty')

	def stopsim(self):
		self.initbutton.config(state="normal", bg='#BFA903')
		self.master.addbutton.config(state="normal")
		self.master.generatemapbutton.config(state="normal")

		for waypoint in self.wpl:
			waypoint.xinput.config(state="normal")
			waypoint.yinput.config(state="normal")
			waypoint.zinput.config(state="normal")
			waypoint.minusbutton.config(state="normal")
			waypoint.upbutton.config(state="normal")
			waypoint.downbutton.config(state="normal")

		self.startstopbutton.config(text="START", bg='green', activebackground='lime', command=self.startsim)
		pass
		#Send SIGINT ou chepakoi

	def landall(self):
		pass
		#Publish un land a tous les drones

	def emergencyland(self):
		pass
		#Publish un emergency land a tous les drones

class waypointinput(Frame):
	def __init__(self,master,waypointlist):
		self.master=master
		self.wpl=waypointlist
		
		Frame.__init__(self, master)
		self.xlabel=Label(self, text="X")
		self.xinput=Entry(self, width=6)
		self.ylabel=Label(self, text="Y")
		self.yinput=Entry(self, width=6)
		self.zlabel=Label(self, text="Z")
		self.zinput=Entry(self, width=6)
		self.minusbutton=Button(self, text="X", command=self.deletewaypoint)
		self.upbutton=self.downbutton=Button(self, text="↑",  command=self.movewaypointup)
		self.downbutton=Button(self,text="↓", command=self.movewaypointdown)

		
		self.xlabel.pack(side=LEFT, padx=4)
		self.xinput.pack(side=LEFT, padx=4)
		self.ylabel.pack(side=LEFT, padx=4)
		self.yinput.pack(side=LEFT, padx=4)
		self.zlabel.pack(side=LEFT, padx=4)
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
		self.waypointlist=[]
		
		Frame.__init__(self, master, borderwidth=2, relief=GROOVE)
		

		self.simframe=simframe(self, self.waypointlist)
		self.simframe.pack()

		self.waypointframe = Frame(self)
		self.waypointcontrolframe = Frame(self)
		
		self.addbutton=Button(self.waypointcontrolframe, text="Add Waypoint",  command=self.addwaypoint)
		self.generatemapbutton=Button(self.waypointcontrolframe, text="Generate Map", command=self.generatemap)
		self.addbutton.pack(side=LEFT, pady=4, fill=BOTH)
		self.generatemapbutton.pack(side=LEFT, pady=4, fill=BOTH)

		self.waypointcontrolframe.pack()

		self.waypointframe.pack(fill=BOTH)

		self.grid(row=0,column=1,sticky='wens')

	def addwaypoint(self):
		waypointinput(self.waypointframe, self.waypointlist)

	def generatemap(self):
		pass
		#generate the map

class term(Frame):
	def __init__(self, master, size, color):
		Frame.__init__(self, master, height=150, width=1500)
		self.wid = self.winfo_id()
		os.system('xterm -into %d -geometry %dx%d -fg %s -sb &' % (self.wid, size[0], size[1], color))

class cameratabmanager(Frame):
	def __init__(self,master,dronelist):
		self.master=master
		self.labellist={}

		Frame.__init__(self, master)

		self.nb = ttk.Notebook(self)
		for drone in dronelist:
			tab = ttk.Frame(self.nb)

			self.nb.add(tab, text=drone['name'])
			self.labellist[drone['name']] = cameradisplay(tab, drone['image'], 0.7)

		self.nb.pack()
		self.grid(row=0,column=0,sticky='wn')

class cameradisplay(Label):
	def __init__(self,master,img,scale):


		self.master = master
		self.image = ImageTk.PhotoImage(img.resize((int(img.size[0]*scale),int(img.size[1]*scale)), Image.ANTIALIAS))
		
		Label.__init__(self, master, image=self.image)
		self.pack()

class mastergui(Frame):
	def __init__(self, master, dronelist):
		self.master = master
		self.dronelist = dronelist
		self.master.title("THE BEST")
		
		Frame.__init__(self, master)
		
		self.tabs = cameratabmanager(self,dronelist)
		
		#waypointsbim = Frame(master)
		self.waypoint = waypointsmanager(self)
		
		"""
		self.terminal = term(self, (246,15), "white")
		self.terminal.grid(row=1,column=0,columnspan=2,sticky='wens')
		
		self.terminal2 = term(self, (246,15),"grey")
		self.terminal2.grid(row=2,column=0,columnspan=2,sticky='wens')
		
		self.terminal3 = term(self, (246,15),"white")
		self.terminal3.grid(row=3,column=0,columnspan=2,sticky='wens')
		"""

if __name__ == "__main__":

	root = Tk()
	dronelist=[
	{'name':'drone1','image': Image.open('mockimg.jpg'), 'ip': '192.168.0.1'},
	{'name':'drone2','image': Image.open('mockimg2.jpg'), 'ip': '192.168.0.2'},
	{'name':'drone3','image': Image.open('mockimg3.jpg'), 'ip': '192.168.0.3'}
	]

	gui = mastergui(root, dronelist)
	gui.pack()

	root.grid_columnconfigure(index=1, weight=2)
	root.grid_rowconfigure(index=1, weight=1)

	while True:
		root.update_idletasks()
		root.update()

		#dronelist update by external process
		dronelist=[
		{'name':'drone1','image': Image.open('mockimg.jpg'), 'ip': '192.168.0.1'},
		{'name':'drone2','image': Image.open('mockimg2.jpg'), 'ip': '192.168.0.2'},
		{'name':'drone3','image': Image.open('mockimg3.jpg'), 'ip': '192.168.0.3'},
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
