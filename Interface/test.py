#! /usr/bin/env python2.7
#-*- coding: utf-8 -*-

from time import sleep
import ttk
from Tkinter import *
from PIL import ImageTk, Image
import os

from drones import dronelist
import rospy
from geometry_msgs.msg import Point
import subprocess

class simframe(Frame):
	def __init__(self, master, waypointlist):
		self.master=master
		self.wpl=waypointlist

		Frame.__init__(self, master)

		self.subprocesslist = []

		self.initbutton=Button(self, text="INIT", bg='#BFA903', fg='white', command=self.initialize)
		self.startstopbutton=Button(self, text="START", bg='green', fg='white', command=self.startsim)
		self.landall=Button(self, text="       Land All       ", command=self.landall)
		self.emergencylandbutton=Button(self, text="ALERT",fg='red', command=self.emergencyland)

		self.initbutton.pack(side=LEFT, pady=4, fill=BOTH)
		self.startstopbutton.pack(side=LEFT, pady=4, fill=BOTH)
		self.landall.pack(side=LEFT, pady=4, fill=BOTH)
		self.emergencylandbutton.pack(side=LEFT, pady=4, fill=BOTH)

	def initialize(self):
		#Init les launch
		self.iplist=''
		
		for process in self.subprocesslist:
			process.terminate()

		del self.subprocesslist[:]


		for drone in self.master.master.dronelist:
			self.iplist+=' '+drone['ip']

			#print('../Launcher_multi_drone/launch_multidrone.sh'+self.iplist)
			#os.system('../Launcher_multi_drone/launch_multidrone.sh'+self.iplist)
			self.subprocesslist.append(subprocess.Popen(['gnome-terminal', '-x', 'roslaunch','bebop_driver','bebop_node_'+ drone['name'] +'.launch'],shell=False))
			rospy.init_node('interface', anonymous= True)

			sub_leader = rospy.Subscriber(drone['name']+'_Pos', Point, lambda msg : refreshposition(msg, self.master.master.dronelist.index(drone)))


	def startsim(self):	
		try:
			"""
			for drone in dronelist:
				os.system('rostopic pub --once '+drone['name']+'/takeoff std_msgs/Empty &')
				print('rostopic pub --once '+drone['name']+'/takeoff std_msgs/Empty')			
			"""
			#os.system('rosrun prise_en_main tele_op_drone.py &')
			#os.system('rostopic pub --once '+drone['name']+'/takeoff std_msgs/Empty &')

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
				self.initbutton.config(state="disabled")
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
		#Publish un land a tous les drones

		
		for drone in dronelist:
			os.system('rostopic pub --once '+drone['name']+'/land std_msgs/Empty &')
			print('rostopic pub --once '+drone['name']+'/land std_msgs/Empty &')
		

	def emergencyland(self):
		#Publish un emergency land a tous les drones

		
		for drone in dronelist:
			os.system('rostopic pub --once '+drone['name']+'/reset std_msgs/Empty &')
			print('rostopic pub --once '+drone['name']+'/reset std_msgs/Empty')
		
		

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

		#Définir l'échelle
			#Connaitre la taille de la Frame
		coursemap=self.master.tabs.coursemap
		coursemap.delete("all")
		framesize=(coursemap.winfo_width(), coursemap.winfo_height())
			#Récupérer les valeur entrées (+0,0 pour les calculs suivants)
		scalelist=[]
		scalelist.append((0,0,0))

		for waypoint in self.waypointlist:
			x=float(waypoint.xinput.get())
			y=float(waypoint.yinput.get())
			z=float(waypoint.zinput.get())
			if(z<1):
				z=1.0
			scalelist.append((x,y,z))
			#Mesurer la distance max entre deux waypoints de la liste (prendre en compte 0,0 aussi)
		maporigin=(0,0)
		mapsize=(0,0)
		for point in scalelist:
			for otherpoint in [scalelist[i] for i in range(scalelist.index(point)+1,scalelist.__len__())]:
					maporigin=(min(otherpoint[0], maporigin[0]), max(otherpoint[1], maporigin[1]))
					mapsize=(max(abs(point[0]-otherpoint[0]),mapsize[0]), max(abs(point[1]-otherpoint[1]),mapsize[1]))

		print(maporigin, mapsize)
			
			#Cette distance = taille de la Frame
			#Rajouter des margins de 1 m sur le bord de la map
		coursemap.xmargin = 30
		coursemap.ymargin = 30
		coursemap.xratio = (framesize[0]-coursemap.xmargin*2)/mapsize[0]
		coursemap.yratio = (framesize[1]-coursemap.ymargin*2)/mapsize[1]
		coursemap.xoffset = -maporigin[0]
		coursemap.yoffset = -maporigin[1]
			
		#Générer la grid avec cette échelle
		coursemap.xaxisstep = 1 #en mètres
		coursemap.yaxisstep = 1 #en mètres

		for xi in range(int(mapsize[0])+1):
			coursemap.create_line(xi*coursemap.xratio+coursemap.xmargin,0,xi*coursemap.xratio+coursemap.xmargin,mapsize[1]*coursemap.yratio+2*coursemap.ymargin, stipple='gray50')

		for yi in range(int(mapsize[1])+1):
			coursemap.create_line(0,yi*coursemap.yratio+coursemap.ymargin,mapsize[0]*coursemap.xratio+2*coursemap.xmargin,yi*coursemap.yratio+coursemap.ymargin, stipple='gray50')

		
		#Créer les waypoins
		pointsize=5

		for point in scalelist:
			if(point[0] == 0 and point[1] == 0):
				coursemap.create_oval(((point[0]+coursemap.xoffset)*coursemap.xratio+coursemap.xmargin)-pointsize, (-(point[1]+coursemap.yoffset)*coursemap.yratio+coursemap.ymargin)-pointsize,((point[0]+coursemap.xoffset)*coursemap.xratio+coursemap.xmargin)+pointsize, (-(point[1]+coursemap.yoffset)*coursemap.yratio+coursemap.ymargin)+pointsize, outline="red", fill="red")
				print(((point[0]+coursemap.xoffset)*coursemap.xratio+coursemap.xmargin)-pointsize, (mapsize[1]-(point[1]+coursemap.yoffset)*coursemap.yratio+coursemap.ymargin)-pointsize,((point[0]+coursemap.xoffset)*coursemap.xratio+coursemap.xmargin)+pointsize, (mapsize[1]-(point[1]+coursemap.yoffset)*coursemap.yratio+coursemap.ymargin)+pointsize)
				print()
			else:
				coursemap.create_oval(((point[0]+coursemap.xoffset)*coursemap.xratio+coursemap.xmargin)-pointsize, (-(point[1]+coursemap.yoffset)*coursemap.yratio+coursemap.ymargin)-pointsize,((point[0]+coursemap.xoffset)*coursemap.xratio+coursemap.xmargin)+pointsize, (-(point[1]+coursemap.yoffset)*coursemap.yratio+coursemap.ymargin)+pointsize, outline="blue", fill="blue")
				print(((point[0]+coursemap.xoffset)*coursemap.xratio+coursemap.xmargin)-pointsize, (mapsize[1]-(point[1]+coursemap.yoffset)*coursemap.yratio+coursemap.ymargin)-pointsize,((point[0]+coursemap.xoffset)*coursemap.xratio+coursemap.xmargin)+pointsize, (mapsize[1]-(point[1]+coursemap.yoffset)*coursemap.yratio+coursemap.ymargin)+pointsize)
	
				#Créer les points des drones (dans la fonction startsim)


		
		

		#generate the map
class mapcanvas(Canvas):
	def __init__(self, master):
		Canvas.__init__(self, master)

		self.xmargin = 0
		self.ymargin = 0
		self.xratio = 0
		self.yratio = 0
		self.xoffset = 0
		self.yoffset = 0
			
		#Générer la grid avec cette échelle
		self.xaxisstep = 1 #en mètres
		self.yaxisstep = 1 #en mètres

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

		self.coursemap = mapcanvas(self.nb)
		self.nb.add(self.coursemap, text="Map")
		
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
		
		
		self.terminal = term(self, (246,15), "white")
		self.terminal.grid(row=1,column=0,columnspan=2,sticky='wens')
		
		self.terminal2 = term(self, (246,15),"grey")
		self.terminal2.grid(row=2,column=0,columnspan=2,sticky='wens')
		
		self.terminal3 = term(self, (246,15),"white")
		self.terminal3.grid(row=3,column=0,columnspan=2,sticky='wens')



def refreshposition(msg,dronenumber):
	print("SALUUUT")
	global dronelist
	dronelist[dronenumber-1]['pos']=(msg.x,msg.y,msg.z)

if __name__ == "__main__":

	root = Tk()

	gui = mastergui(root, dronelist)
	gui.pack()

	root.grid_columnconfigure(index=1, weight=2)
	root.grid_rowconfigure(index=1, weight=1)


	for drone in dronelist:
		drone['oval']=gui.tabs.coursemap.create_oval(drone['pos'][0]-5,drone['pos'][1]-5,drone['pos'][0]+5,drone['pos'][1]+5, outline="black", fill="green")



	#sub_leader = rospy.Subscriber("/bebop1_Pos", Point, refreshposition1)#bebop 1 is leader

	while True:
		root.update_idletasks()
		root.update()

		#Drone reload on canvas
		for drone in dronelist:
			gui.tabs.coursemap.delete(drone['oval'])
			#gui.tabs.coursemap.create_oval(((drone['pos'][0]+gui.tabs.coursemap.xoffset)*gui.tabs.coursemap.xratio+gui.tabs.coursemap.xmargin)-1, (-(drone['pos'][1]+gui.tabs.coursemap.yoffset)*gui.tabs.coursemap.yratio+gui.tabs.coursemap.ymargin)-1,((drone['pos'][0]+gui.tabs.coursemap.xoffset)*gui.tabs.coursemap.xratio+gui.tabs.coursemap.xmargin)+1, (-(drone['pos'][1]+gui.tabs.coursemap.yoffset)*gui.tabs.coursemap.yratio+gui.tabs.coursemap.ymargin)+1, outline="green", fill="green")
			drone['oval']=gui.tabs.coursemap.create_oval(((drone['pos'][0]+gui.tabs.coursemap.xoffset)*gui.tabs.coursemap.xratio+gui.tabs.coursemap.xmargin)-5, (-(drone['pos'][1]+gui.tabs.coursemap.yoffset)*gui.tabs.coursemap.yratio+gui.tabs.coursemap.ymargin)-5,((drone['pos'][0]+gui.tabs.coursemap.xoffset)*gui.tabs.coursemap.xratio+gui.tabs.coursemap.xmargin)+5, (-(drone['pos'][1]+gui.tabs.coursemap.yoffset)*gui.tabs.coursemap.yratio+gui.tabs.coursemap.ymargin)+5, outline="black", fill="green")

			#gui.tabs.coursemap.itemconfig(drone['oval'], fill="blue")

		"""
		#Tabs hot reload
		if not len(dronelist) is len(gui.tabs.labellist):
			print('Faire un hotreload des tabs ici')

		"""

		"""
		#Camera hot reload
		imagetemp={}
		for drone in dronelist:
			print(drone['name'] == gui.tabs.nb.tab(gui.tabs.nb.select(), "text"))
			if drone['name'] == gui.tabs.nb.tab(gui.tabs.nb.select(), "text"):
				imagetemp[drone['name']] = ImageTk.PhotoImage(drone['image'].resize((int(drone['image'].size[0]*0.7),int(drone['image'].size[1]*0.7)), Image.ANTIALIAS))
				gui.tabs.labellist[drone['name']].configure(image=imagetemp[drone['name']])
		"""


