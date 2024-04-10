import pygame
from constants import *
from random import random


class myApp():
    def __init__(self, title, width, height, dx):
        pygame.init()
        pygame.font.init()
        self.size = 30
        self.font = pygame.font.SysFont('Comic Sans MS', self.size)
        self.app = pygame
        self.width_ = width
        self.height_ = height
        self.title = title
        self.screen = pygame.display
        self.screen.set_caption(title)
        self.canvas = self.screen.set_mode((width,height))
        self.running = True
        self.frame = 0
        self.dx = dx
        self.dy = dx
        self.rows = int(height/dx)
        self.cols = int(width/dx)
        self.grid = [[False for j in range(self.cols)] for i in range(self.rows)]
        self.mouse_pos = (0,0)
        print('Rows: ' + str(self.rows))
        print('Cols: ' + str(self.cols))
        self.start = (0,0)
        self.end = (self.rows-1,self.cols-1)
        self.path = []
        self.hasSolution = False
        self.iterations = 10
        self.is_fullscreen = False
        self.visualization = True
        self.diagonals = False

    def update(self):
        inputs = {
            'mouse_click': (0,0,0),
            'mouse_wheel': -1
        }
        for event in pygame.event.get():
            inputs['mouse_click'] = pygame.mouse.get_pressed()
            if inputs['mouse_click'] != (0,0,0):
                if inputs['mouse_click'] == (0,1,0):
                    self.grid = [[False for j in range(self.cols)] for i in range(self.rows)]
                else:
                    mouse_p = pygame.mouse.get_pos()
                    mouse_p = (int(mouse_p[0]/self.dy),int(mouse_p[1]/self.dx))
                    if mouse_p != self.mouse_pos:
                        self.mouse_pos = mouse_p
                        self.pen(self.mouse_pos, inputs['mouse_click'])
            
            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 4:
                    inputs['mouse_wheel'] = 'Up'
                    self.iterations += 1
                if event.button == 5 and self.iterations > 1:
                    inputs['mouse_wheel'] = 'Down'
                    self.iterations += -1


            if event.type == pygame.KEYDOWN:
                self.key_press(event)
                
            if event.type == pygame.QUIT:
                self.running = False

        self.show()
        self.screen.update()
        return inputs

    def key_press(self, event):
        if event.key == 13: #Run A* Search Path Finding Algorithm
            self.Astar(self.start,self.end)
        if event.key == 292 or event.key == 1073741892: #Togglem Fullscreen Mode
            self.fullscreen()
        if event.key == 27: #Exit App
            if self.is_fullscreen:
                self.fullscreen()
            self.running = False
        if event.key == 118: #Toggle Visualization
            self.visualization = not(self.visualization)
        if event.key == 100: #Toggle Diagonals
            self.diagonals = not(self.diagonals)
        if event.key == 114:
            self.grid = [[random() < 0.30 * (1 + 2/3*int(self.diagonals)) and (i,j)!=self.start and (i,j)!=self.end for j in range(self.cols)] for i in range(self.rows)]

    def fullscreen(self):
        self.is_fullscreen = not(self.is_fullscreen)
        if self.is_fullscreen:
            self.canvas = self.screen.set_mode((GetSystemMetrics(0),GetSystemMetrics(1)), pygame.FULLSCREEN)
        else:
            self.canvas = self.screen.set_mode((self.width_,self.height_))

        self.size = self.screen.get_surface().get_size()
        self.dx = self.size[0] / self.cols
        self.dy = self.size[1] / self.rows



    def show(self):
        #Show Walls
        for i in range(self.rows):
            for j in range(self.cols):
                if self.grid[i][j]:
                    pygame.draw.rect(self.canvas, (255,255,255), (j*self.dx,i*self.dy,self.dx,self.dy))
                else:
                    pygame.draw.rect(self.canvas, (0,0,0), (j*self.dx,i*self.dy,self.dx,self.dy))
        
        #Show Path Solution
        if self.hasSolution:
            for point in self.path:
                #if not self.grid[point[1]][point[0]]:
                pygame.draw.rect(self.canvas, (242, 227, 10), (point[1]*self.dx,point[0]*self.dy,self.dx,self.dy))

        #Show Start and End Points
        pygame.draw.rect(self.canvas, (0,255,0), (self.start[1]*self.dx,self.start[0]*self.dy,self.dx,self.dy))
        pygame.draw.rect(self.canvas, (255,0,0), (self.end[1]*self.dx,self.end[0]*self.dy,self.dx,self.dy))

        #Show Properties
        text = [
            'Speed: ' + str(self.iterations),
            'Visualization: ' + str(self.visualization),
            'Diagonals: ' + str(self.diagonals)
        ]
        y = self.screen.get_surface().get_size()[1]
        for t in text[::-1]:
            text_surface = self.font.render(t, True, (255,255,255))
            h = self.font.size(t)[1]
            y += -h
            self.canvas.blit(text_surface, (0,y))
    
    def pen(self, mouse_pos, mouse_click):
        self.grid[mouse_pos[1]][mouse_pos[0]] = int(mouse_click == (1,0,0))

    def Astar(self, start, goal):
        def h(start, goal):
            if self.diagonals:
                return abs(start[0]-goal[0]) + abs(start[1]-goal[1])
            else:
                return (start[0]-goal[0])**2+(start[1]-goal[1])**2

        def reconstruct_path(current, start):
            path = []
            while current != start:
                path.append(current)
                current = cameFrom[current]
            return path

        self.path = []
        
        G = {}
        F = {}

        G[start] = 0
        F[start] = G[start] + h(start, goal)

        cameFrom = {}
        closedSet = {}
        openSet = {}
        openSet[start] = 'open'
        neighbors = {}
        current = start

        win = False

        while len(openSet)>0:
            for it in range(self.iterations):
                
                #Find  in openSet the point with lowest F = G + h
                current = min(openSet, key = lambda k : F[k])
                    
                #If has arrived to the goal reconstruct path and done
                if current == goal:
                    win = True
                    self.path = reconstruct_path(current, self.start)
                    self.hasSolution = True
                    break

                del openSet[current]
                closedSet[current] = 'close'

                #Get neighbours
                possible_relative_neighbors = [(di,dj) for di in range(-1,2) for dj in range(-1,2) if (di!=0 or dj!=0) and (di==0 or dj==0 or self.diagonals)]
                
                neighbors = []
                for di,dj in possible_relative_neighbors:
                    neighbor = (current[0]+di,current[1]+dj)
                    if neighbor[0]>=0 and neighbor[0]<self.rows and neighbor[1]>=0 and neighbor[1]<self.cols and not(neighbor in closedSet) and not(self.grid[neighbor[0]][neighbor[1]]):
                        neighbors.append(neighbor)
                        new_g = G[current] + int(10*(di**2+dj**2)**0.5)/10 #current.move_cost(node)
                        #Otherwise if it is already in the open set
                        if neighbor in openSet:
                            #Check if we beat the G score 
                            if G[neighbor] > new_g:
                                #If so, update the node to have a new parent
                                G[neighbor] = new_g
                                cameFrom[neighbor] = current
                        else:
                            #If it isn't in the open set, calculate the G and H score for the node
                            G[neighbor]= new_g
                            F[neighbor] = G[neighbor] + h(neighbor, goal)
                            #Set the parent to our current item
                            cameFrom[neighbor] = current
                            #Add it to the set
                            openSet[neighbor] = 'open'

                
                if len(openSet) == 0:
                    break
            

            if self.visualization: #Visualizing
                self.update()
                for vert in closedSet:
                    pygame.draw.rect(self.canvas, (255,0,0),(vert[1]*self.dx,vert[0]*self.dy,self.dx,self.dy))
                for vert in openSet:
                    pygame.draw.rect(self.canvas, (0,255,255),(vert[1]*self.dx,vert[0]*self.dy,self.dx,self.dy))
                self.screen.update()
                pygame.time.delay(10)
            
            if win or len(openSet) == 0:
                if self.visualization:
                    pygame.time.delay(500)
                break
        
        if win:
            #Show Walls
            for i in range(self.rows):
                for j in range(self.cols):
                    if self.grid[i][j]:
                        pygame.draw.rect(self.canvas, (255,255,255), (j*self.dx,i*self.dy,self.dx,self.dy))
                    else:
                        pygame.draw.rect(self.canvas, (0,0,0), (j*self.dx,i*self.dy,self.dx,self.dy))
                        
            #Show Start and End Points
            pygame.draw.rect(self.canvas, (0,255,0), (self.start[1]*self.dx,self.start[0]*self.dy,self.dx,self.dy))
            pygame.draw.rect(self.canvas, (255,0,0), (self.end[1]*self.dx,self.end[0]*self.dy,self.dx,self.dy))

            #Show Properties
            text = [
                'Speed: ' + str(self.iterations),
                'Visualization: ' + str(self.visualization),
                'Diagonals: ' + str(self.diagonals)
            ]
            y = self.screen.get_surface().get_size()[1]
            for t in text[::-1]:
                text_surface = self.font.render(t, True, (255,255,255))
                h = self.font.size(t)[1]
                y += -h
                self.canvas.blit(text_surface, (0,y))
            
            #Show Path
            for point in self.path[::-1]:
                pygame.draw.rect(self.canvas, (242, 227, 10), (point[1]*self.dx,point[0]*self.dy,self.dx,self.dy))
                self.screen.update()
                pygame.time.delay(10)
            
            print('Done')
        else:
            print('fail')




myApp = myApp('A* Path Finding Algorithm', WIDTH, HEIGHT, DX)


while myApp.running:
    inputs = myApp.update()
    if inputs['mouse_click'] == (0,0,0) and inputs['mouse_wheel'] == -1:
        pygame.time.delay(100)
