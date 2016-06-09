from __future__ import division
from Tkinter import *
import time, math, StringIO, subprocess




master = Tk()
w = Canvas(master, width=1000, height=1000)
w.pack()
def main():
	minx = 0
	miny = 0
	boundaries = []
	robotPos = [75,100] #this will be read in
	inputBounds = [10,10, 10,100, 100,100, 100,10] #these will be read in at the start of the program
	boundaries,minx, miny = adjustCoords(inputBounds)
	homepos = [minx , miny]
	#robotPos[0] += minx *-1
	#robotPos[1] += miny *-1
	home = w.create_oval(homepos[0]-3, homepos[1]-3, homepos[0]+3, homepos[1]+3, fill ="green")
	robot = Robot(robotPos[0],robotPos[1],homepos,home)
	movex = 0
	movey = 0

	for i in range (0, len(boundaries)//2):
		if ( i+1  == len(boundaries)//2):
			w.create_line(boundaries[(i*2) ],boundaries[(i*2) +1],boundaries[0], boundaries[1])
			

		else:
			w.create_line(boundaries[(i*2)], boundaries[(i*2)+1], boundaries[(i*2) +2], boundaries [(i*2)+3])
	
	robot.path(boundaries)
	while True:
		line,movex,movey = robot.inBounds(inBounds(robot.getPos(), boundaries))
		if (robot.getBounds() == True):
			print "IN BOUNDS"
		else: 
			print "OUT OF BOUNDS"
		w.update()
		robot.updatePos(movex,movey)
		robot.draw()
		time.sleep(0.01)
		w.delete(line)


	mainloop()


def line (p1, p2):

    A = (p1[1] - p2[1])
    B = (p2[0] - p1[0])
    C = (p1[0] * p2[1] - p2[0] * p1[1])
    return A, B, -C

def intersection(L1, L2):
    D  = L1[0] * L2[1] - L1[1] * L2[0]
    Dx = L1[2] * L2[1] - L1[1] * L2[2]
    Dy = L1[0] * L2[2] - L1[2] * L2[0]
    if D != 0:
        x = Dx / D
        y = Dy / D
        return x, y
    else:
        return False

def adjustCoords (boundArray): #boundArray should be a list of positions defining a border
	minx = 100 
	miny = 100
	out = [0,0]
	for i in range (0, len(boundArray)//2):
		if (boundArray[i*2] < 0 or boundArray[i*2] < minx):
			minx = boundArray[i*2]
		if (boundArray[(i*2) + 1] < 0 or boundArray[(i*2) + 1] < miny):
			miny = boundArray[i*2 +1] 
	#for i in range (0, len(boundArray)//2):
	##	boundArray[i*2 + 1] += ((miny  * -1) +10)
	
	return (boundArray, minx, miny)


def inBounds(robotPos,boundaryLines):
	robotLines = line((robotPos[0],1000), (robotPos[0],-1000))
	robotLinesy = line((-1000, robotPos[1]), (1000, robotPos[1]))
	Bline = (0,0,0)
	count = 0
	Blines = []
	BlinesY = []
	intersections = []
	intersectionsY =[]
	for i in range (0, len(boundaryLines)//2):
		if(i + 1 == len(boundaryLines)//2):
			if ((robotPos[0]<=boundaryLines[0] or robotPos[0]<=boundaryLines[i*2]) and (robotPos[0]>=boundaryLines[0] or robotPos[0]>=boundaryLines[i*2])):
				Bline = line((boundaryLines[0],boundaryLines[1]),(boundaryLines[i*2],boundaryLines[i*2+1]))
				Blines.append(Bline)

			if ((robotPos[1]<=boundaryLines[1] or robotPos[1]<=boundaryLines[i*2+1]) and (robotPos[1]>=boundaryLines[1] or robotPos[1]>=boundaryLines[i*2+1])):
				Bline = line((boundaryLines[0],boundaryLines[1]),(boundaryLines[i*2],boundaryLines[i*2+1]))
				BlinesY.append(Bline)

		else: 
			if ((robotPos[0]<=boundaryLines[i*2] or robotPos[0]<=boundaryLines[(i*2) +2]) and (robotPos[0]>=boundaryLines[i*2] or robotPos[0]>=boundaryLines[i*2+2])) :
				Bline = line((boundaryLines[(i*2)],boundaryLines[(i*2)+1]),(boundaryLines[(i*2)+2],boundaryLines[(i*2)+3]))
				Blines.append(Bline)

			if ((robotPos[1]<=boundaryLines[i*2 + 1] or robotPos[1]<=boundaryLines[(i*2) +3]) and (robotPos[1]>=boundaryLines[i*2 + 1] or robotPos[1]>=boundaryLines[i*2+3])) :
				Bline = line((boundaryLines[(i*2)],boundaryLines[(i*2)+1]),(boundaryLines[(i*2)+2],boundaryLines[(i*2)+3]))
				BlinesY.append(Bline)


	for j in range (0, len(Blines)):
		R = intersection(robotLines,Blines[j])
		if R:
			count = count + 1
			intersections.append(R)

	for i in range (0, len(BlinesY)):
		R = intersection(robotLinesy,BlinesY[i])
		if R:
			intersectionsY.append(R)

	check = 0
	checkY = 0
	for i in range(0,len(intersections)//2):
		intersections.sort()
		if (intersections[i*2][1]<= robotPos[1] and intersections[i*2 + 1][1]>=robotPos[1]):
			#return True
			check = 1
			break

	for i in range(0,len(intersectionsY)//2):
		intersectionsY.sort()
		if (intersectionsY[i*2][0]<=robotPos[0] and intersectionsY[i*2 + 1][0]>=robotPos[0]):
			#return True
			checkY = 1
			break

	if (check == 1 or checkY == 1):
		return True
	else:
		return False
				
class Robot:

	def __init__(self, positionx, positiony, home, homecircle):
		self.posx = positionx
		self.posy = positiony
		self.robo = w.create_oval(self.posx-3, self.posy-3, self.posx+3, self.posy+3, fill ="yellow")
		self.inbounds = True
		self.points = []
		self.allPoints = []
		self.circles = []
		self.line = w.create_line(0,0,0,0)
		self.count = 0
		self.home = home
		homex = home[0]
		homey = home[1]
		self.number = 0
		self.homeCircle = homecircle

	def updatePos(self,positionx, positiony):
		self.posx += positionx
		self.posy += positiony

	def inBounds(self, status):
		shortest = 10000000
		index = 0
		movex = 0
		movey = 0
		line = 0
		self.inbounds = status
		if self.number == self.count:
			print "[Course covered]"
		else:
			if self.inbounds == False:
				for i in range(0, len(self.allPoints)):

					distance = math.sqrt((self.posx - self.allPoints[i][0])**2 +  (self.posy - self.allPoints[i][1])**2)
					if distance < shortest:
						shortest = distance
						index = i
				line = w.create_line(self.posx, self.posy, self.allPoints[index][0],self.allPoints[index][1])
				movex = -(self.posx - self.allPoints[index][0])/shortest
				movey = -(self.posy - self.allPoints[index][1])/shortest
				
			
			else: 
				line = w.create_line(self.posx, self.posy, self.points[0][0],self.points[0][1])
				shortest = math.sqrt((self.posx - self.points[0][0])**2 +  (self.posy - self.points[0][1])**2)
				movex = -(self.posx - self.points[0][0])/shortest
				movey = -(self.posy - self.points[0][1])/shortest
				if shortest <= 1:
					self.points.pop(0)
					w.delete(self.circles.pop(0))
					self.count = self.count + 1
				if len (self.circles) == 70:
					textBattery = w.create_text(50, 120, text = "Battery Low", fill = "red")
				if len (self.circles) == 54:
					textBattery1 = w.create_text(50, 120, text = "Battery Low", fill = "white")
		return line,movex,movey

	def getBounds (self):
		return self.inbounds

	def draw(self):
		w.coords(self.robo, self.posx-3, self.posy-3, self.posx+3, self.posy+3)#check this

	def getPos(self):
		robotPos = [self.posx,self.posy]
		return robotPos
	def path(self, boundaryLines):
		reverse = True
		for i in range (0 ,200):
			reverse = not(reverse)
			if reverse:
				for j in range(0 , 200):
					if i == 5 and j == 7:
						pos = [10,10]
					else:
						pos = [i*10,j*10]
					if (inBounds(pos, boundaryLines)):
						point = w.create_oval((i*10)+1, (j*10)+1, (i*10) -1 , (j*10)-1, fill ="red")
						self.points.append(pos)
						self.circles.append(point)
						self.allPoints.append(pos)
			else:
				for j in range(200 , 0,-1):
					pos = [i*10,j*10]
					if (inBounds(pos, boundaryLines)):
						point = w.create_oval((i*10)+1, (j*10)+1, (i*10) -1 , (j*10)-1, fill ="red")
						self.points.append(pos)
						self.circles.append(point)
						self.allPoints.append(pos)
		self.points.append(self.home)
		self.circles.append(self.homeCircle)
		
		self.number = len(self.points)
	#def pipe(self):
#
#		output = self.allPoints
#		p = subprocess.Popen(['test.c'], shell=True, stdin = subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
#		p.stdout.write(output)
		#output = p.communicate()[0]
		#print output

		#for i in range (0, len(self.allPoints)):
		#	print (self.allPoints)
		#output.write()
		#for 
		#self.allPoints
main()