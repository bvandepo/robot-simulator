#!/usr/bin/python3 
#Bertrand VANDEPORTAELE 2018
#tiré de https://www.linux.com/blog/python-stl-model-loading-and-display-opengl

#doc pyglet
#https://pyglet.readthedocs.io/en/pyglet-1.3-maintenance/programming_guide/quickstart.html

#mélange de opengl3.py et pyglettest2.py

#bvandepo@rapid:~/Téléchargements/reprap2/boulot_bvdp/support_ueye_et_pixy$ cp support_ueye_stereo_V2_tube_circulaire.stl ~/Bureau/pythonb/test.stl



import statistics
# Use: statistics.mean(liste)
from statistics import *
# Use: mean(liste)


'''
#positionnement fenêtre dans quart supérieur gauche pour évier les problèmes de rafraichissement sur écran 4k
#https://www.pygame.org/wiki/SettingWindowPosition
import os
x = 200
y = 20
os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (x,y)
'''


import os
import struct


from pyglet.gl import *
from pyglet.window import *
import math

from OpenGL.GL import *
from OpenGL.GLU import *
import pygame
from pygame.locals import *
########################################################################################################################
#class for a 3d point
class createpoint:
    def __init__(self,p,c=(1,0,0)):
        self.point_size=0.5
        self.color=c
        self.x=p[0]
        self.y=p[1]
        self.z=p[2]

    def glvertex(self):
        glVertex3f(self.x,self.y,self.z)
########################################################################################################################
#class for a 3d face on a model
class createtriangle:
    points=None
    normal=None
    def __init__(self, p1, p2, p3, n=None):
        #3 points of the triangle
        #print(' createtriangle '+ str(n))
        self.points=createpoint(p1),createpoint(p2),createpoint(p3)
        if n==None:
          #compute triangles normal
          #print(' compute triangles normal')
          self.normal=createpoint(self.calculate_normal(self.points[0],self.points[1],self.points[2]))#(0,1,0)#
        else:
          #print(' using existing triangles normal')
          #print ('type(n):'+str(type(n)))    #-> type(n):<class 'tuple'>
          normal = (n[0], n[1], n[2])
          #print ('type(normal):'+str(type(normal)))
          self.normal=normal
          #self.normal = createpoint(self.calculate_normal(self.points[0], self.points[1], self.points[2]))  # (0,1,0)#

    #calculate vector / edge
    def calculate_vector(self,p1,p2):
        return -p1.x+p2.x,-p1.y+p2.y,-p1.z+p2.z

    def calculate_normal(self,p1,p2,p3):
        a=self.calculate_vector(p3,p2)
        b=self.calculate_vector(p3,p1)
        #calculate the cross product returns a vector
        return self.cross_product(a,b)

    def cross_product(self,p1,p2):
        return (p1[1]*p2[2]-p2[1]*p1[2]) , (p1[2]*p2[0])-(p2[2]*p1[0]) , (p1[0]*p2[1])-(p2[0]*p1[1])
########################################################################################################################
class loader:

    def __init__(self):  # ,stlfilename):
        self.model=[]
        self.batch=None
    #return the faces of the triangles
    def get_triangles(self):
        if self.model:
            for face in self.model:
                yield face #returns a generator
                #https://stackoverflow.com/questions/231767/what-does-the-yield-keyword-do

    def generate_batch(self):
        vertices = []
        normals = []
        for tri in self.get_triangles():
            #glNormal3f(tri.normal[0], tri.normal[1], tri.normal[2])
            vertices.append(tri.points[0].x)
            vertices.append(tri.points[0].y)
            vertices.append(tri.points[0].z)
            vertices.append(tri.points[1].x)
            vertices.append(tri.points[1].y)
            vertices.append(tri.points[1].z)
            vertices.append(tri.points[2].x)
            vertices.append(tri.points[2].y)
            vertices.append(tri.points[2].z)
            for p in range(3): #add a normal vector for each vertex
                normals.append(tri.normal[0])
                normals.append(tri.normal[1])
                normals.append(tri.normal[2])
        self.batch = pyglet.graphics.Batch()  # liste de commandes précalculées pour accélérer le rendu
        vertex_count = len(vertices)//3
        self.vertex_list = self.batch.add( vertex_count , pyglet.gl.GL_TRIANGLES, None, ('v3f', vertices),('n3f', normals) )

    def draw(self):
    #draw the models faces
        if self.batch==None:
            self.drawSlow()
        else:
            self.drawBatch()

    def drawBatch(self):
        self.batch.draw()

    def drawSlow(self):
        glBegin(GL_TRIANGLES)
        for tri in self.get_triangles():
            glNormal3f(tri.normal[0], tri.normal[1], tri.normal[2])
            glVertex3f(tri.points[0].x,tri.points[0].y,tri.points[0].z)
            glVertex3f(tri.points[1].x,tri.points[1].y,tri.points[1].z)
            glVertex3f(tri.points[2].x,tri.points[2].y,tri.points[2].z)
        glEnd()

    #generate test model with triangles
    def generate(self):
        print( "generate test model with triangles")
        triangle=[]
        normal=(0,0,1)
        triangle.append((0.,0.,0.))
        triangle.append((0.,1.,0.))
        triangle.append((1.,0.,0.))
        self.model.append(createtriangle(triangle[0],triangle[1],triangle[2],normal))



    #load stl file detects if the file is a text file or binary file
    def load_stl(self,filename):
        #read start of file to determine if its a binay stl file or a ascii stl file
        fp=open(filename,'rb')
        h=fp.read(80)
        type=h[0:5]
        fp.close()
        #print('entete: '+str(type))
        if type==b'solid':
            print( "reading text file"+str(filename))
            self.load_text_stl(filename)
        else:
            print( "reading binary stl file "+str(filename,))
            self.load_binary_stl(filename)
        self.generate_batch()

    #read text stl match keywords to grab the points to build the model
    def load_text_stl(self,filename):
        fp=open(filename,'r')
        #listallpointsx=[]
        #listallpointsy=[]
        #listallpointsz=[]
        normal =None #no normal by default

        for line in fp.readlines():
            words=line.split()
            #print('words: '+str(words))
            #print('words[0]: '+str(words[0]))

            if len(words)>0:
                if words[0]=='solid':
                    self.name=words[1]

                if words[0]=='facet':
                    center=[0.0,0.0,0.0]
                    triangle=[]
                    normal=(float(eval(words[2])),float(eval(words[3])),float(eval(words[4]))) #cast en float sinon les valeurs entières sont stockées en int
                    #print('normal: ' + str(normal) ) #words[2]) + '  ' +  str(words[3]) + '  ' +  str(words[4])  )
                if words[0]=='vertex':
                    triangle.append((float(eval(words[1])),float(eval(words[2])),float(eval(words[3])))) #cast en float sinon les valeurs entières sont stockées en int
                    #listallpointsx.append(eval(words[1]))
                    #listallpointsy.append(eval(words[2]))
                    #listallpointsz.append(eval(words[3]))
                if words[0]=='endloop':
                    #make sure we got the correct number of values before storing
                    #print('len(triangle): ' + str(len(triangle)))
                    if len(triangle)==3:
                        #print('add triangle: ' + str(triangle[0]) + str(triangle[1]) +str(triangle[2])  +str(normal) )
                        #print('normal triangle: ' + str(normal))
                        self.model.append(createtriangle(triangle[0], triangle[1], triangle[2], normal))
                        #calcul de la normale
                        #t=createtriangle(triangle[0], triangle[1], triangle[2], normal)
                        #normal2=t.calculate_normal(self, p1, p2, p3):
                        normal = None  # no normal by default for the next triangle

        #print('listallpoints:' +str(listallpointsx))
        #moyennex=mean(listallpointsx)
        #moyenney=mean(listallpointsy)
        #moyennez=mean(listallpointsz)
        #print('moyenne: '+str(moyennex) + ' , '+str(moyenney) + ' , '+str(moyennez))
        #maximum=max(listallpointsx)
        #print(maximum)
        print('number of triangles: ' + str(len(self.model)))
        fp.close()

    #load binary stl file check wikipedia for the binary layout of the file
    #we use the struct library to read in and convert binary data into a format we can use
    def load_binary_stl(self,filename):
        fp=open(filename,'rb')
        h=fp.read(80)

        l=struct.unpack('I',fp.read(4))[0]
        count=0
        while True:
            try:
                p=fp.read(12)
                if len(p)==12:
                    n=struct.unpack('f',p[0:4])[0],struct.unpack('f',p[4:8])[0],struct.unpack('f',p[8:12])[0]

                p=fp.read(12)
                if len(p)==12:
                    p1=struct.unpack('f',p[0:4])[0],struct.unpack('f',p[4:8])[0],struct.unpack('f',p[8:12])[0]

                p=fp.read(12)
                if len(p)==12:
                    p2=struct.unpack('f',p[0:4])[0],struct.unpack('f',p[4:8])[0],struct.unpack('f',p[8:12])[0]

                p=fp.read(12)
                if len(p)==12:
                    p3=struct.unpack('f',p[0:4])[0],struct.unpack('f',p[4:8])[0],struct.unpack('f',p[8:12])[0]

                new_tri=(n,p1,p2,p3)

                if len(new_tri)==4:
                    tri=createtriangle(p1,p2,p3,n)
                    self.model.append(tri)
                count+=1
                fp.read(2)

                if len(p)==0:
                    break
            except EOFError:
                break
        fp.close()

########################################################################################################################
'''
class draw_scene:
    def __init__(self,style=1):
        #create a model instance and
        self.model1=loader()

        self.model1.load_stl(os.path.abspath('')+'/test.stl')
#        self.model1.generate()

        self.init_shading()
        self.cpt=0


    #solid model with a light / shading
    def init_shading(self):
        glShadeModel(GL_SMOOTH)
        glClearColor(0.0, 0.0, 0.0, 0.0)
        glClearDepth(1.0)
        glEnable(GL_DEPTH_TEST)
        glShadeModel(GL_SMOOTH) 
        glDepthFunc(GL_LEQUAL)
        glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)
      
        glEnable(GL_COLOR_MATERIAL)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)   
        glLight(GL_LIGHT0, GL_POSITION,  (0, 1, 1, 0))      
        glMatrixMode(GL_MODELVIEW)
      
    def resize(self,width, height):
        if height==0:
            height=1
        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, 1.0*width/height, 0.1, 100.0)
        #gluLookAt(0.0,0.0,45.0,0,0,0,0,40.0,0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

    def init(self):
        glShadeModel(GL_SMOOTH)
        glClearColor(0.0, 0.0, 0.0, 0.0)
        glClearDepth(1.0)
        glEnable(GL_DEPTH_TEST)
        glShadeModel(GL_SMOOTH) 
        glDepthFunc(GL_LEQUAL)
        glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)
        glEnable(GL_COLOR_MATERIAL)
      
#sactivation éclairage
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)   
#        glLight(GL_LIGHT0, GL_POSITION,  (0, 1, 1, 0))
        glLight(GL_LIGHT0, GL_POSITION,  (0, 10, 10, 0))

        glMatrixMode(GL_MODELVIEW)

    def draw(self):
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
      
#        glTranslatef(0.0,-26.0, -100.0)
        gluLookAt( 10.,10.,20., 0.,0.,0., 0.,1.,0.)
 
        glRotatef(self.cpt, 0, 1, 0)
        print('self.cpt: '+str(self.cpt))
        self.cpt=(self.cpt+1 )%360

#        fact=1000
#        glScalef(fact,fact,fact)
        self.model1.draw()
'''
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################
########################################################################################################################


class Model:
    '''
    # https://groups.google.com/forum/#!topic/pyglet-users/HSuYh8hQ2cw
    def create_circle(x, y, radius, batch):
        circle, indices = create_indexed_vertices(x, y, radius)
        vertex_count = len(circle) // 2
        vertex_list = self.batch.add_indexed(vertex_count, pyglet.gl.GL_TRIANGLES, None,
                                             indices,
                                             ('v2f', circle),
                                             ('c4f', (1, 1, 1, 0.8) * vertex_count))

    def create_indexed_vertices(x, y, radius, sides=24):
        vertices = [x, y]
        for side in range(sides):
            angle = side * 2.0 * math.pi / sides
            vertices.append(x + math.cos(angle) * radius)
            vertices.append(y + math.sin(angle) * radius)
        # Add a degenerated vertex
        vertices.append(x + math.cos(0) * radius)
        vertices.append(y + math.sin(0) * radius)
        indices = []
        for side in range(1, sides + 1):
            indices.append(0)
            indices.append(side)
            indices.append(side + 1)
        return vertices, indices
    '''
    def get_tex(self, file):
        tex = pyglet.image.load(file).texture
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
        return pyglet.graphics.TextureGroup(tex)

    def __init__(self): #,stlfilename):
        #degrés de liberté du modèle:

        self.d1=0 #self.cpt
        self.d2=0 #self.cpt
        self.d3=0
        self.d4=0
        #d5 = -100 + ($t * 120); # d5 = -100;
        self.d5=-100 #+(self.cpt %120);


        self.cpt = 0
        #self.init_shading()
        #self.top = self.get_tex('grass_top.png')
        #self.side = self.get_tex('grass_side.png')
        #self.bottom = self.get_tex('dirt.png')
        #self.batch = pyglet.graphics.Batch()
        # https://pythonhosted.org/pyglet/api/pyglet.graphics.Batch-class.html
        tex_coords = ('t2f', (0, 0, 1, 0, 1, 1, 0, 1,))


#TODO: passer les model en batch pour accélérer le rendu ou glDrawArrays

        # create  model instances
        nbparts=9
        self.model_list = []
        for i in range(1,nbparts+1):
          self.model_list.append(loader())
          filename= os.path.abspath('') +'/model/stl/p'+str(i)+'.stl'
          print(str(i) +'=> load '+filename)
          self.model_list[i-1].load_stl(filename)  #stockage dans la liste à partir de l'indice 0

#        #self.model1.generate()
        #self.model1.load_stl(os.path.abspath('') + '/test.stl')
        #self.model1.load_stl(os.path.abspath('') + '/test3.stl')



    #        self.batch = pyglet.graphics.Batch()
    #        for i in range(2):
    #          create_circle(window.width*random.random(), window.height*random.random(), 20, batch)

    # face ajoutée
    # https://pyglet.readthedocs.io/en/pyglet-1.3-maintenance/modules/graphics/
    #        vertex_list = pyglet.graphics.vertex_list(3,
    #                                          ('v2f', (0.0, 1.0, 1.0, 0.0, 1.0, 1.0)),
    #                                          ('c4B', (255, 255, 255, 255) * 3))
    #        self.batch.add(3,GL_TRIANGLES,('v2f', (0.0, 1.0, 1.0, 0.0, 1.0, 1.0)), ('c4B', (255, 255, 255, 255) * 3))

    # https://leovt.wordpress.com/2015/10/04/render-to-texture-with-python-3-and-pyglet/

    #        self.batch.add(3,GL_TRIANGLES,("c4B",(255,0,0,255)),('v3f',(-2,-2,0,   2,-2,0,  2,2,0   )) )

    #        self.batch.add(4,GL_QUADS,self.side,('v3f',(-2,-2,0,   2,-2,0,  2,2,0,   -2,2,0, )),tex_coords)
    #        x,y,z = 0,0,-1
    #        X,Y,Z = x+1,y+1,z+1
    #        self.batch.add(4,GL_QUADS,self.side,('v3f',(x,y,z, x,y,Z, x,Y,Z, x,Y,z, )),tex_coords)
    #        self.batch.add(4,GL_QUADS,self.side,('v3f',(X,y,Z, X,y,z, X,Y,z, X,Y,Z, )),tex_coords)
    #        self.batch.add(4,GL_QUADS,self.bottom,('v3f',(x,y,z, X,y,z, X,y,Z, x,y,Z, )),tex_coords)
    #        self.batch.add(4,GL_QUADS,self.top,('v3f',(x,Y,Z, X,Y,Z, X,Y,z, x,Y,z, )),tex_coords)
    #        self.batch.add(4,GL_QUADS,self.side,('v3f',(X,y,z, x,y,z, x,Y,z, X,Y,z, )),tex_coords)
    #        self.batch.add(4,GL_QUADS,self.side,('v3f',(x,y,Z, X,y,Z, X,Y,Z, x,Y,Z, )),tex_coords)

    #solid model with a light / shading
    def init_rendering(self):
        #print('initializing rendering')
        glEnable(GL_CULL_FACE)
        glShadeModel(GL_SMOOTH)
        #depth= 0 à znear et 1 à zfar:  https://learnopengl.com/Advanced-OpenGL/Depth-testing
        glEnable(GL_DEPTH_TEST)
        glDepthFunc(GL_LESS)
        glClearDepth(1.0)
        glDepthMask(GL_TRUE)
        glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST) #Indicates the quality of color, texture coordinate, and fog coordinate interpolation. If perspective-corrected parameter interpolation is not efficiently supported by the GL implementation, hinting GL_DONT_CARE or GL_FASTEST can result in simple linear interpolation of colors and/or texture coordinates.
        glEnable(GL_COLOR_MATERIAL)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glLight(GL_LIGHT0, GL_POSITION,  (20, 20, 20, 0))
        #explications éclairage: https: // www.khronos.org / opengl / wiki / How_lighting_works
        glLight(GL_LIGHT0, GL_AMBIENT, (0.5, 0.5, 0.5, 1))
        glLight(GL_LIGHT0, GL_DIFFUSE, (0.4, 0.4, 0.4, 1))
        glLight(GL_LIGHT0, GL_SPECULAR, (0.1, 0.1, 0.1, 1))
        glMatrixMode(GL_MODELVIEW)
        glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)
        glEnable(GL_COLOR_MATERIAL)
        #https://www.khronos.org/registry/OpenGL-Refpages/gl2.1/xhtml/glColorMaterial.xml
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE )
        glMatrixMode(GL_MODELVIEW)
        #print('finished initializing rendering')

    def draw(self):
        self.init_rendering()
        glRotatef(-90, 1, 0, 0);
        self.cpt = self.cpt + 1
        '''        glTranslatef(0.5, 0.5, 0.5);
        glRotatef(self.cpt, 0, 1, 0);
        glTranslatef(-0.5, -0.5, -0.5);
        '''

        # self.batch.draw()

        # ici
        #self.model1.draw()
#        for i in range(1,9):
#         self.model_list[i-1].draw()
#
#       for model in self.model_list:
#            model.draw()
        #self.model_list[5-1].draw()


        #translation pour chaque éléments de la pince
        l5 = self.d5 / 12;

        glRotatef(-90, 0, 0, 1)

        glColor3f(1.0,1.0,1.0)
        self.model_list[1-1].draw()
        glRotatef(self.d1,0, 0, 1)
        glTranslatef(0, 0, 28)
        glColor3f(1.0,0.0,0.0)
        self.model_list[2-1].draw()

        glTranslatef(0, 0, 13)
        glRotatef(self.d2, 0, 1, 0)
        glColor3f(0.0, 1.0, 0.0)
        self.model_list[3-1].draw()

        glTranslatef(49.5, 0, 0)
        glRotatef(self.d3,0, 1, 0)
        glColor3f(0.0, 0.0, 1.0)
        self.model_list[9-1].draw()

        glTranslatef(49.5, 0, 0)
        glRotatef(self.d4,0, 1, 0)
        glColor3f(0.0, 1.0, 1.0)
        self.model_list[4 - 1].draw()

        glTranslatef(0, 0, -8)
        glColor3f(1.0, 1.0, 0.0)
        glPushMatrix(); #1
        glTranslatef(45, 18, -2)
        glRotatef(90, 0, 1, 0)
        glRotatef(180, 1, 0, 0)
        self.model_list[5 - 1].draw()

        glPopMatrix(); #0
        glPushMatrix();#1
        glTranslatef(0, -l5, 0)
        glPushMatrix();#2
        glTranslatef(81, -15.5, 20)
        glRotatef(180, 0, 1, 0)
        glRotatef(90, 1, 0, 0)
        self.model_list[7 - 1].draw()

        glPopMatrix();#1
        glTranslatef(58, -22.5, 18)
        glRotatef(90, 0, 1, 0)
        self.model_list[8 - 1].draw()

        glPopMatrix();#0
        glPushMatrix();#1
        glTranslatef(0, +l5, 0)
        glPushMatrix();#2
        glTranslatef(81, 3.5, -3)
        glRotatef(180, 0, 1, 0)
        glRotatef(-90, 1, 0, 0)
        self.model_list[7 - 1].draw()

        glPopMatrix();#1
        glPushMatrix(); #2
        glTranslatef(58, 12.5, -1)
        glRotatef(180, 0, 0, 1)
        glRotatef(-90, 0, 1, 0)
        self.model_list[8 - 1].draw()
        glPopMatrix();  # 1
        glPopMatrix();  # 0

        glColor3f(1.0, 0.0, 1.0)
        glTranslatef(61, -5, 8.5)
        glRotatef(self.d5, 1, 0, 0)
        self.model_list[6 - 1].draw()



        triangle1 = False
        if triangle1:
            # affichage d'un triangle non texturé seul
            #       https://www.youtube.com/watch?v=TGqtZ-LK_fU
            vertices = [0.0, 1.0, 1.0, 0.0, 1.0, 1.0]
            vertices_gl = (GLfloat * len(vertices))(*vertices)
            glEnableClientState(GL_VERTEX_ARRAY)
            glVertexPointer(2, GL_FLOAT, 0, vertices_gl)
            glDrawArrays(GL_TRIANGLES, 0, int(len(vertices) / 2))
        triangle2 = False
        if triangle2:
            pyglet.graphics.draw(3, pyglet.gl.GL_TRIANGLES, ('v2f', (0.0, 1.0, 1.0, 0.0, 1.0, 1.0)),
                             ('c3B', (255, 255, 255, 255, 0, 0, 0, 0, 255)))
            pyglet.graphics.draw(3, pyglet.gl.GL_TRIANGLES, ('v3f', (0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 1.0, 1.0, 2.0)),
                             ('c3B', (255, 255, 255, 255, 0, 0, 0, 255, 0)))

    def update(self, dt , keys):
        dtmul=dt*5
        if keys[key.T]: self.d1 += dtmul
        if keys[key.G]: self.d1 -= dtmul
        if keys[key.Y]: self.d2 += dtmul
        if keys[key.H]: self.d2 -= dtmul
        if keys[key.U]: self.d3 += dtmul
        if keys[key.J]: self.d3 -= dtmul
        if keys[key.I]: self.d4 += dtmul
        if keys[key.K]: self.d4 -= dtmul
        if keys[key.O]: self.d5 += dtmul*4
        if keys[key.L]: self.d5 -= dtmul*4
        #TODO: gérer des méthodes de pilotage des axes avec vitesse min/maxi et butées


########################################################################################################################

class Player:
    def __init__(self, pos=(0, 0, 0), rot=(0, 0)):
        self.pos = list(pos)
        self.rot = list(rot)
        self.deltascroll=0
        self.speedscroll=5
    def mouse_motion(self, dx, dy):
        dx /= 8;
        dy /= 8;
        self.rot[0] += dy;
        self.rot[1] -= dx
        if self.rot[0] > 90:
            self.rot[0] = 90
        elif self.rot[0] < -90:
            self.rot[0] = -90
    def mouse_scroll(self, dy):
        self.deltascroll+=dy

    def update(self, dt, keys):
        s = dt * 10
        rotY = -self.rot[1] / 180 * math.pi
        dx, dz = s * math.sin(rotY), s * math.cos(rotY)
        #gestion scrolling souris
        self.pos[0] += self.deltascroll*self.speedscroll*dx;
        self.pos[2] -= self.deltascroll*self.speedscroll*dz;
        self.deltascroll=0;

        if keys[key.W]: self.pos[0] += dx; self.pos[2] -= dz
        if keys[key.S]: self.pos[0] -= dx; self.pos[2] += dz
        if keys[key.A]: self.pos[0] -= dz; self.pos[2] -= dx
        if keys[key.D]: self.pos[0] += dz; self.pos[2] += dx

        if keys[key.SPACE]: self.pos[1] += s
        if keys[key.LSHIFT]: self.pos[1] -= s

        print('player : ' +str(self.pos) +str(self.rot))





########################################################################################################################

class Window(pyglet.window.Window):
    def setMatrix(self, pos, rot):
        glRotatef(-rot[0], 1, 0, 0); glRotatef(-rot[1], 0, 1, 0); glTranslatef(-pos[0], -pos[1], -pos[2], )
    def Projection(self):
        glMatrixMode(GL_PROJECTION); glLoadIdentity()
    def Model(self):
        glMatrixMode(GL_MODELVIEW); glLoadIdentity()
    def set2d(self):
        self.Projection(); gluOrtho2D(0, self.width, 0, self.height); self.Model()
    def set3d(self):
        self.Projection(); gluPerspective(70, self.width / self.height, 0.1, 1000); self.Model()
    def setLock(self, state):
        self.lock = state; self.set_exclusive_mouse(state)

    lock = False;
    mouse_lock = property(lambda self: self.lock, setLock)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.fps_display = FPSDisplay(self)
        self.set_minimum_size(300, 200)
        self.keys = key.KeyStateHandler()
        self.push_handlers(self.keys)
        pyglet.clock.schedule(self.update)
        #self.player = Player((0.5, 1.5, 10.5), (-30, 0))
        self.player = Player((18.9301446246206, 1.5, 189.66455050743974), (36. , 15.75))

    #utilisation souris
#https://pyglet.readthedocs.io/en/pyglet-1.3-maintenance/programming_guide/mouse.html

    def set_Model(self,model):
        self.model = model

    def on_mouse_scroll(self, x, y, scroll_x, scroll_y):
        """Zoom centré sur la souris.
        :param x: Abscisse de la souris.
        :type x: int
        :param y: Ordonnée de la souris.
        :type y: int
        :param scroll_x: Scroll horizontal de la souris (rare).
        :type scroll_x: int
        :param scroll_y: Scroll vertical ('clics' de molette).
        :type scroll_y: int
        """
        #if self.mouse_lock:
        self.player.mouse_scroll(scroll_y)
        #print('scroll_y '+str(scroll_y))

    def on_mouse_press(self, x, y, button, modifiers):
        """Dessine des cellules (efface avec <ctrl>).
        :param x: Abscisse de la souris.
        :type x: int
        :param y: Ordonnée de la souris.
        :type y: int
        :param button: Code de la touche appuyée.
        :type button: int
        :param modifiers: Combinaison des codes des touches spéciales.
        :type modifiers: int
        """
        if button & pyglet.window.mouse.LEFT:
           self.mouse_lock= not self.mouse_lock
    '''def on_mouse_release(x, y, button, modifiers):
        if button & pyglet.window.mouse.LEFT:
            self.mouse_lock =False
'''
    def on_mouse_motion(self, x, y, dx, dy):
        if self.mouse_lock: self.player.mouse_motion(dx, dy)

    def on_key_press(self, KEY, MOD):
        if KEY == key.ESCAPE:
            self.close()
        elif KEY == key.E:
            self.mouse_lock = not self.mouse_lock

    def update(self, dt):
        self.player.update(dt, self.keys)
        self.model.update(dt, self.keys)

    def on_draw(self):
        glClearColor(0.5, 0.7, 1, 1)
        self.clear()
        self.set3d()
        glPushMatrix()
        self.setMatrix(self.player.pos, self.player.rot)
        self.model.draw()
        glPopMatrix()
        self.fps_display.draw()


########################################################################################################################

if __name__ == '__main__':
    model = Model() #'/test3.stl')
    window = Window(width=854, height=480, caption='OpenGL Python B.Vandeportaele', resizable=True)
    window.set_location(20, 20)
    window.set_Model(model)
    secondWindow=False
    if secondWindow:
        window2 = Window(width=654, height=480, caption='OpenGL Python B.Vandeportaele 2', resizable=True) #, vsync=False)
        window2.set_location(1060, 20)
        window2.set_Model(model)

    pyglet.app.run()


'''
#main program loop
def main(): 
    #initalize pygame
    pygame.init()
    pygame.display.set_mode((640,480), OPENGL|DOUBLEBUF)

    #setup the open gl scene
    scene=draw_scene()
    scene.resize(640,480)
  
    frames = 0
    ticks = pygame.time.get_ticks()
    while 1:
        event = pygame.event.poll()
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            break

       
        #draw the scene
        scene.draw() 
        pygame.display.flip()
        pygame.time.wait(10)
        frames = frames+1

        #always reset this windows as the active one.... only for linux
        #command='wmctrl -a \''+windowsName+'\''
        #os.system(command)

        x = 100
        y = 0
        os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (x,y)

    print( "fps:  %d" % ((frames*1000)/(pygame.time.get_ticks()-ticks)))


if __name__ == '__main__':
    main()
'''