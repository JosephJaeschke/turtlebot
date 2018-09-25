class mapData(object):
	def __init__(self,edges,obstacles,paths,dx,dy):
		self.length=dx #number of cells in x direction
		self.height=dy #number of cells in y direction
		self.corners=edges
		self.obsList=obstacles
		self.sgPairs=paths
		miny=99999
		self.origin=0
		for i in self.corners:
			if i[1]<miny:
				miny=i[1]
		topLeft=99999
		for i in self.corners:
			if i[0]+miny-i[1]<topLeft:
				self.origin=i

class Vertex(object):
	def __init__(self,point):
		self.pos=point #coord of bottom left corner of cell
		self.g=0
		self.h=0
		self.parent=None

class Cell(object):
	def __init__(self,pos,vert,full):
		self.pos=pos	#tuple with x and y cell index
		self.vertex=vert #each cell has the 0-n index while the Vertex has the coordinate
		self.blocked=full

import numpy as np
import pygame as pg
import heapq
import os.path
import time
import math

grid=None
mData=None
width=0.2 #size of every cell (square) in meters
window=0

def pointToIndex(point):
	#converts a tuple holding a coordinate to a tuple index for grid
	return (round((point[0]-mData.origin[0])/width),round((point[1]-mData.origin[1])/width))

def indexToPoint(index):
	#converts a tuple holding an index to a tuple coord of the bottom left corner of cell
	return (round(mData.origin[0]+width*index[0],1),round(mData.origin[1]-width*(index[1]+1),1))

def isBlocked(x1,y1):
	#returns true if there is an obstacle in a given index tuple, false otherwise
	basePt=indexToPoint((x1,y1))
	boundary=[basePt,(basePt[0]+width,basePt[1]),(basePt[0],basePt[1]+width),(basePt[0]+width,basePt[1]+width)]
	#if on edge
	if x1==0 or x1==mData.length-1 or y1==0 or y1==mData.height-1:
		return True
	#if obstacle
	l=list(mData.obsList)
	for o in l:
		#for every obstacle
		for point in o:
		#for every point of the obstacle
			tempObsList=list(o)
			tempObsList.remove(point)
			for otherPt in tempObsList:
				#find line to other point in obstacle
				#print point, otherPt
				#print point[0]-otherPt[0]
				if point[0]-otherPt[0]==0 and (abs(point[0]-basePt[0])<=0.1 or abs(point[0]-basePt[0]-width)<=0.1):
					if min(point[1],otherPt[1])<=basePt[1]<=max(point[1],otherPt[1]) or min(point[1],otherPt[1])<=basePt[1]+width<=max(point[1],otherPt[1]):
						return True
					continue
				elif point[0]-otherPt[0]==0:
					continue
				m=(point[1]-otherPt[1])/(point[0]-otherPt[0])
				if m==0 and (abs(point[1]-basePt[1])<=0.1 or abs(point[1]-basePt[1]-width)<=0.1):
					if min(point[0],otherPt[0])<=basePt[0]<=max(point[0],otherPt[0]) or min(point[0],otherPt[0])<=basePt[0]+width<=max(point[0],otherPt[0]):
						return True
					continue
				elif point[1]-otherPt[1]==0:
					continue
				#check if intersect four line segments of cell
				#top
				x=((boundary[0][1]+width-point[1])/m)+point[0]
				if basePt[0]<=x<=basePt[0]+width and min(otherPt[1],point[1])<=basePt[1]+width<=max(otherPt[1],point[1]):
					return True
				#right
				y=m*(basePt[0]+width-point[0])+point[1]
				if basePt[1]<=y<=basePt[1]+width and min(otherPt[0],point[0])<=basePt[0]+width<=max(otherPt[0],point[0]):
					return True
				#bottom
				x=((boundary[0][1]-point[1])/m)+point[0]
				if basePt[0]<=x<=basePt[0]+width and min(otherPt[1],point[1])<=basePt[1]<=max(otherPt[1],point[1]):
					return True
				#left
				y=m*(basePt[0]-point[0])+point[1]
				if basePt[1]<=y<=basePt[1]+width and min(otherPt[0],point[0])<=basePt[0]<=max(otherPt[0],point[0]):
					return True
	return False

def chbyshvDist(pos,end):
	#Given the position tuple and end tuple 
	return max(abs(pos[0]-end[0]),abs(pos[1]-end[1]))
	

def gridSolver(heur,update,strt,goal):
	#A* function takes a heursitic function and getParent function to switch between A* and FDA*
	#strt is the tuple for the inital position (given in actual coordinates)
	#end is the tuple for the goal (given in actual coordinates)
	path=[]		#list of coordinates in optimal path
	fringe=[]	#list of coordinates in open list
	closed=[]	#list of coordinates in closed list
	curPos=(heur(strt,goal),strt) #tuples in heap are coordinates, not indexes
	curIndex=pointToIndex(strt)
	heapq.heappush(fringe,curPos) #push start into heap. Heapq orders tuples by first element
	while fringe: #while fringe is non empty
		curPos=heapq.heappop(fringe)
		curIndex=pointToIndex(curPos[1])
		if curPos==goal:
			print "Found Path!"
			nodeIndex=pointToIndex(curPos[1])
			node=grid[nodeIndex[0]][nodeIndex[1]]
			while not(node.vertex.parent==None):
				path.append(node.vertex.pos) #make a list of corrdinate tuples
				parent=node.vertex.parent
				parentIndex=pointToIndex(parent)
				node=grid[parentIndex[0]][parentIndex[1]]
		row=curIndex[0]
		col=curIndex[1]
		closed.append(curPos[1])
		succ=[] #all successors of curPos
		#check top right neighbor
		if grid[r][c].blocked==False:
			succ.append((r+1,c-1))
		#check top left neighbor
		if r-1>=0:
			if grid[r-1][c].blocked==False:
				succ.append((r-1,c-1))
		#check bottom right neighbor
		if c+1<mData.height:
			if grid[r][c+1].blocked==False:
				succ.append((r+1,c+1))
		#check bottom left neighbor
		if r-1>=0 and c+1<mData.length:
			if grid[r-1][c+1].blocked==False:
				succ.append((r-1,c+1))
		#check right neighbor
		if c+1<mData.height:
			if grid[r][c].blocked==False and grid[r][c+1].blocked==False:
				succ.append((r+1,c))
		#check top neighbor
		if r-1>=0:
			if grid[r][c].blocked==False and grid[r-1][c].blocked==False:
				succ.append((r,c-1))
		#check left neighbor
		if r-1>0 and c+1<mData.height:
			if grid[r-1][c].blocked==False and grid[r-1][c+1].blocked==False:
				succ.append((r-1,c))
		#check down neighbor
		if r-1>0 and c+1<mData.height:
			if grid[r-1][c+1].blocked==False and grid[r][c+1].blocked==False:
				succ.append((r+1,c+1))
		for x in succ:
			#check if it exists, then if it's in the closed list, then if it's in the fringe
			if x[0]<0 or x[0]>=mData.length or x[1]<0 or x[1]>=mData.height:
				continue
			if x not in closed:
				inF=[a for a,b in enumerate(fringe) if y[1]==x]
				if inF==[]:
					#not in fringe so add it
					grid[x[0]][x[1]].vertex.g=float("inf")




def printMap():
	global window
	pg.display.set_caption("grid")
	window=pg.display.set_mode((mData.length*5,mData.height*5))
	pg.font.init()
	font=pg.font.SysFont("monospace",5)
	pg.init()
	pg.display.set_caption("grid")
	window=pg.display.set_mode((mData.length*5,mData.height*5))
	bgc=(0,0,0)
	window.fill(bgc)
	bs=5
	for i in range(mData.height):
		for j in range(mData.length):
			rect=pg.Rect(j*bs,i*bs,bs,bs)
			if grid[j][i].blocked==True:
				pg.draw.rect(window,(0,0,255),rect)
			else:
				pg.draw.rect(window,(255,255,255),rect)
	for i in range(mData.length):
		pg.draw.rect(window,(0,0,0),(i*5,0,1,mData.height*5),1)
	for j in range(mData.height):
		pg.draw.rect(window,(0,0,0),(0,j*5,mData.length*5,1),1)
	pg.display.update()
	

if __name__ == "__main__":
	#parse map file
	print "Enter map file"
	while True:
		fpath=raw_input()
	 	if os.path.exists(fpath):
			break
		print "ERROR: No such file"
	f=open(fpath,"r")
	corners=f.readline()
	corners=corners.split()
	obstacles=f.readline()
	oList=[]
	obstacles=f.readline()
	while obstacles != "---\n":
		oList.append(obstacles.split())
		obstacles=f.readline()
	pathPair=f.readline()
	pathList=[]
	while pathPair != "":
		pathList.append(pathPair.split())
		pathPair=f.readline()
	f.close()

	#initalize map metadata structure
	corners=[(tuple(map(float,x[1:-1].split(",")))) for x in corners]
	tempObjList=[]
	for obs in oList:
		tempObjList.append([(tuple(map(float,x[1:-1].split(",")))) for x in obs])
	oList=tempObjList
	tempPairList=[]
	for p in pathList:
		tempPairList.append([(tuple(map(float,x[1:-1].split(",")))) for x in p])
	pathList=tempPairList
	xDist=int(math.ceil(math.ceil(abs(corners[0][0])+abs(corners[2][0]))/width))
	yDist=int(math.ceil(math.ceil(abs(corners[0][1])+abs(corners[2][1]))/width))
	mData=mapData(corners,oList,pathList,xDist,yDist)
	#initalize map grid
	grid=np.zeros([mData.length,mData.height],object) #[row,col]
	for x in range(mData.length):
		for y in range(mData.height):
			v=Vertex(indexToPoint((x,y)))
			grid[x][y]=Cell((x,y),v,isBlocked(x,y))
	printMap()
	while True:
		time.sleep(1)
