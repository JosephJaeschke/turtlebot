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
fPts=[]

def pointToIndex(point):
	#converts a tuple holding a coordinate to a tuple index for grid
	return (int(round((point[0]-mData.origin[0])/width)),int(round(((point[1]-mData.origin[1])/(-1*width))-1)))

def indexToPoint(index):
	#converts a tuple holding an index to a tuple coord of the bottom left corner of cell
	return (round(mData.origin[0]+width*index[0],1),round(mData.origin[1]-width*(index[1]+1),1))

def dist(pt1,pt2):
	p1=indexToPoint(pt1)
	p2=indexToPoint(pt2)
	return math.sqrt((p2[0]-p1[0])*(p2[0]-p1[0])+(p2[1]-p1[1])*(p2[1]-p1[1]))

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
	coord=indexToPoint(pos)
	return max(abs(coord[0]-end[0]),abs(coord[1]-end[1]))

def lineOfSight(gparent,child):
	x0=gparent[0]
	y0=gparent[1]
	x1=child[0]
	x2=child[1]
	f=0
	dx=x1-x0
	dy=y1-y0
	sx=1
	sy=1
	if dy<0:
		dy=-1*dy
		sy=-1
	if dx<0:
		dx=-1*dx
		sx-1
	if dx>=dy:
		while x0!=x1:
			f=f+dy
			if f>=dx:
				if grid[x0+((sx-1)/2)][y0+((sy-1)/2)].blocked:
					return False
				y0=y0+sy
				f=f-dx
			if f!=0 and grid[x0+((sx-1)/2)][y0+((sy-1)/2)].blocked:
				return False
			if dy==0 and grid[x0+((sx-1)/2)][y0].blocked and grid[x0+((sx-1)/2)][y0-1].blocked:
				return False
			x0=x0+sx
	else:
		while y0!=y1:
			f=f+dx
			if f>=dy:
				if grid[x0+((sx-1)/2)][y0+((sy-1)/2)].blocked:
					return False
				x0=x0+sx
				f=f-dy
			if f!=0 and grid[x0+((sx-1)/2)][y0+((sy-1)/2)].blocked:
				return False
			if dx==0 and grid[x0][y0+((sy-1)/2)].blocked and grid[x0-1][y0+((sy-1)/2)].blocked:
				return False
			y0=y0+sy
	return True


def updateFDA_star(parent,child,fringe,heur,goal):
	gparent=grid[parent[0]][parent[1]].vertex.parent
	if lineOfSight(vertex,child):
		cost=dist(gparent,child)
		if grid[gparent[0]][gparent[1]].vertex.g+cost<grid[child[0]][child[1]].vertex.g:
			grid[child[0]][child[1]].vertex.g=grid[gparent[0]][gparent[1]].vertex.g+cost
			grid[child[0]][child[1]].vertex.parent=gparent
			if not inF==[]:
				fringe.pop(inF[0])
				heapq.heapify(fringe)
			grid[child[0]][child[1]].vertex.h=heur(child,goal)
			heapq.heappush(fringe,(grid[child[0]][child[1]].vertex.g+grid[child[0]][child[1]].vertex.h,child))
			fPts.append(child)


	else:
		cost=dist(parent,child)
		if grid[parent[0]][parent[1]].vertex.g+cost<grid[child[0]][child[1]].vertex.g:
			grid[child[0]][child[1]].vertex.g=grid[parent[0]][parent[1]].vertex.g+cost
			grid[child[0]][child[1]].vertex.parent=parent
			inF=[a for a,b in enumerate(fringe) if b[1]==child]
			if not inF==[]:
				fringe.pop(inF[0])
				heapq.heapify(fringe)
			grid[child[0]][child[1]].vertex.h=heur(child,goal)
			heapq.heappush(fringe,(grid[child[0]][child[1]].vertex.g+grid[child[0]][child[1]].vertex.h,child))
			fPts.append(child)


	
def updateA_star(parent,child,fringe,heur,goal):
	cost=dist(parent,child)
	'''
	if parent[0]-child[0]==0 != parent[1]-child[1]==0: #not a diagonal
		cost=0.2
	else:
		cost=1.414*0.2
	'''
	if grid[parent[0]][parent[1]].vertex.g+cost<grid[child[0]][child[1]].vertex.g:
		grid[child[0]][child[1]].vertex.g=grid[parent[0]][parent[1]].vertex.g+cost
		grid[child[0]][child[1]].vertex.parent=parent
		inF=[a for a,b in enumerate(fringe) if b[1]==child]
		if not inF==[]:
			fringe.pop(inF[0])
			heapq.heapify(fringe)
		grid[child[0]][child[1]].vertex.h=heur(child,goal)
		heapq.heappush(fringe,(grid[child[0]][child[1]].vertex.g+grid[child[0]][child[1]].vertex.h,child))
		fPts.append(child)

	

def gridSolver(heur,update,strt,goal):
	#A* function takes a heursitic function and getParent function to switch between A* and FDA*
	#strt is the tuple for the inital position (given in actual coordinates)
	#end is the tuple for the goal (given in actual coordinates)
	path=[]		#list of coordinates in optimal path
	fringe=[]	#list of coordinates in open list
	closed=[]	#list of coordinates in closed list
	curIndex=(heur(strt,goal),pointToIndex(strt)) #tuples in heap indexes
	heapq.heappush(fringe,curIndex) #push start into heap. Heapq orders tuples by first element
	while fringe: #while fringe is non empty
		curIndex=heapq.heappop(fringe)
		curIndex=curIndex[1]
		if curIndex==pointToIndex(goal):
			print "Found Path!"
			node=grid[curIndex[0]][curIndex[1]]
			while not(node.vertex.parent==None):
				path.append(node.vertex.pos) #make a list of corrdinate tuples
				parentIndex=node.vertex.parent
				node=grid[parentIndex[0]][parentIndex[1]]
			return path
		r=curIndex[0]
		c=curIndex[1]
		closed.append(curIndex)
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
		#check dowon neighbor
		if r-1>0 and c+1<mData.height:
			if grid[r-1][c+1].blocked==False and grid[r][c+1].blocked==False:
				succ.append((r,c+1))
		for x in succ:
			#check if it exists, then if it's in the closed list, then if it's in the fringe
			if x[0]<0 or x[0]>=mData.length or x[1]<0 or x[1]>=mData.height:
				continue
			if x not in closed:
				inF=[a for a,b in enumerate(fringe) if b[1]==x]
				if inF==[]:
					#not in fringe so add it
					grid[x[0]][x[1]].vertex.g=float("inf")
					grid[x[0]][x[1]].vertex.parent=None
				update((r,c),x,fringe,heur,goal)
	print "No path..."
	return None


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
	strt=pointToIndex(mData.sgPairs[0][0])
	end=pointToIndex(mData.sgPairs[0][1])
	for x in fPts:
		pg.draw.rect(window,(100,0,100),(x[0]*bs,x[1]*bs,bs,bs))
	pg.draw.rect(window,(255,0,0),(end[0]*bs,end[1]*bs,bs,bs),1)
	pg.draw.rect(window,(0,255,0),(strt[0]*bs,strt[1]*bs,bs,bs),1)
	pg.display.update()

def printPath(path):
	if path==None:
		return
	global window
	for coord in path:
		p=pointToIndex(coord)
		print p
		rect=pg.Rect(p[0]*5,p[1]*5,5,5)
		pg.draw.rect(window,(255,255,0),rect)
	for i in range(mData.length):
		pg.draw.rect(window,(0,0,0),(i*5,0,1,mData.height*5),1)
	for i in range(mData.height):
		pg.draw.rect(window,(0,0,0),(0,i*5,mData.length*5,1),1)
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
	p=gridSolver(chbyshvDist,updateA_star,mData.sgPairs[0][0],mData.sgPairs[0][1])
	printMap()
	printPath(p)
	while True:
		time.sleep(1)
