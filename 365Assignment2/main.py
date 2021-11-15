# Triangle strips
#
# Usage: python main.py file_of_triangles
#
# You can press ESC in the window to exit.
#
# You'll need Python 3 and must install these packages:
#
#   PyOpenGL, GLFW
import random
import sys, os, math, time


try: # PyOpenGL
  from OpenGL.GL import *
except:
  print( 'Error: PyOpenGL has not been installed.' )
  sys.exit(0)

try: # GLFW
  import glfw
except:
  print( 'Error: GLFW has not been installed.' )
  sys.exit(0)



# Globals

window = None

windowWidth  = 1000 # window dimensions
windowHeight = 1000

minX = None # range of vertices
maxX = None
minY = None
maxY = None

r  = 0.008 # point radius as fraction of window size

allVerts = [] # all triangle vertices

lastKey = None  # last key pressed

showForwardLinks = True



# Triangle
#
# A Triangle stores its three vertices and pointers to any adjacent triangles.
#
# For debugging, you can set the 'highlight1' and 'highlight2' flags
# of a triangle.  This will cause the triangle to be highlighted when
# it's drawn.


class Triangle(object):

    nextID = 0

    def __init__( self, verts ):

      self.verts   = verts # 3 vertices.  Each is an index into the 'allVerts' global.
      self.adjTris = [] # adjacent triangles

      self.nextTri = None  # next triangle on strip
      self.prevTri = None  # previous triangle on strip

      self.highlight1 = False # to cause drawing to highlight this triangle in colour 1
      self.highlight2 = False # to cause drawing to highlight this triangle in colour 2

      self.centroid = ( sum( [allVerts[i][0] for i in self.verts] ) / len(self.verts),
                        sum( [allVerts[i][1] for i in self.verts] ) / len(self.verts) )

      self.id = Triangle.nextID
      Triangle.nextID += 1


    # String representation of this triangle
    
    def __repr__(self):
        return 'tri-%d' % self.id


    # Draw this triangle
    
    def draw(self):

        # Highlight with yellow fill

        if self.highlight1 or self.highlight2:

            if self.highlight1:
                glColor3f( 0.9, 0.9, 0.4 ) # dark yellow
            else:
                glColor3f( 1, 1, 0.8 ) # light yellow

            glBegin( GL_POLYGON )
            for i in self.verts:
                glVertex2f( allVerts[i][0], allVerts[i][1] )
            glEnd()

        # Outline the triangle

        glColor3f( 0, 0, 0 )
        glBegin( GL_LINE_LOOP )
        for i in self.verts:
            glVertex2f( allVerts[i][0], allVerts[i][1] )
        glEnd()


    # Draw edges to next and previous triangle on the strip

    def drawPointers(self):

        if showForwardLinks and self.nextTri:
            glColor3f( 0, 0, 1 )
            drawArrow( self.centroid[0], self.centroid[1], 
                       self.nextTri.centroid[0], self.nextTri.centroid[1] )

        if not showForwardLinks and self.prevTri:
            glColor3f( 1, 0, 0 )
            drawArrow( self.centroid[0], self.centroid[1], 
                       self.prevTri.centroid[0], self.prevTri.centroid[1] )

        if not self.nextTri and not self.prevTri: # no links.  Draw a dot.
            if showForwardLinks:
                glColor3f( 0, 0, 1 )
            else:
                glColor3f( 1, 0, 0 )
            glBegin( GL_POLYGON )
            for i in range(100):
                theta = 3.14159 * i/50.0
                glVertex2f( self.centroid[0] + 0.5 * r * math.cos(theta), self.centroid[1] + 0.5 * r * math.sin(theta) ) 
            glEnd()


    # Determine whether this triangle contains a point
    
    def containsPoint( self, pt ):

        return (turn( allVerts[self.verts[0]], allVerts[self.verts[1]], pt ) == LEFT_TURN and
                turn( allVerts[self.verts[1]], allVerts[self.verts[2]], pt ) == LEFT_TURN and
                turn( allVerts[self.verts[2]], allVerts[self.verts[0]], pt ) == LEFT_TURN)
      



# Draw an arrow between two points.

def drawArrow( x0,y0, x1,y1 ):

    d = math.sqrt( (x1-x0)*(x1-x0) + (y1-y0)*(y1-y0) )

    vx = (x1-x0) / d      # unit direction (x0,y0) -> (x1,y1)
    vy = (y1-y0) / d

    vpx = -vy             # unit direction perpendicular to (vx,vy)
    vpy = vx

    xa = x0 + 0.15*r*vx # arrow tail
    ya = y0 + 0.15*r*vy

    xb = x1 - 0.15*r*vx # arrow head
    yb = y1 - 0.15*r*vy

    xc = xb - 2*r*vx + 0.5*r*vpx # arrow outside left
    yc = yb - 2*r*vy + 0.5*r*vpy

    xd = xb - 2*r*vx - 0.5*r*vpx # arrow outside right
    yd = yb - 2*r*vy - 0.5*r*vpy

    glBegin( GL_LINES )
    glVertex2f( xa, ya )
    glVertex2f( 0.5*(xc+xd), 0.5*(yc+yd) )
    glEnd()

    glBegin( GL_LINE_LOOP )
    glVertex2f( xb, yb )
    glVertex2f( xc, yc )
    glVertex2f( xd, yd )
    glEnd()
      
      

# Determine whether three points make a left or right turn

LEFT_TURN  = 1
RIGHT_TURN = 2
COLLINEAR  = 3

def turn( a, b, c ):

    det = (a[0]-c[0]) * (b[1]-c[1]) - (b[0]-c[0]) * (a[1]-c[1])

    if det > 0:
        return LEFT_TURN
    elif det < 0:
        return RIGHT_TURN
    else:
        return COLLINEAR



# ================================================================
# ================================================================
# ================================================================

# Build a set of triangle strips that cover all of the given
# triangles.  The goal is to make the strips as long as possible
# (i.e. to have the fewest strip that cover all triangles).
#
# Follow the instructions in A2.txt.
#
# This function does not return anything.  The strips are formed by
# modifying the 'nextTri' and 'prevTri' pointers in each triangle.

#function to pick best triangle from remaining non-strip triangles to start the next strip at
def findFirst(Tris):
    #set placeholder variables to none
    first1=None
    first2=None
    first3=None
    other=None
    currx=100
    pastx=100
    curry=100
    pasty=100
    #for every triangle in the copy list, check to find the triangle with the least number of valid adjacent triangels,
    #then return that value. For only the value of one adjacent triangle, if found the loop breaks before getting
    #throught the rest of the list.
    for p in Tris:
        if ((len(p.adjTris)==1)and(p.adjTris[0].nextTri==None)):
            first1=p
            break
        elif((len(p.adjTris)==2)and((p.adjTris[0].nextTri==None)or(p.adjTris[1].nextTri==None))):
            if ((p.adjTris[0].nextTri==None)and(p.adjTris[1].nextTri==None)):
                currx=2
            else:
                currx=1
            if (currx<pastx):
                first2=p
                pastx=currx
        elif((len(p.adjTris)==3)and((p.adjTris[0].nextTri==None)or(p.adjTris[1].nextTri==None)or(p.adjTris[2].nextTri==None))):
            if ((p.adjTris[0].nextTri==None)and(p.adjTris[1].nextTri==None)and(p.adjTris[2].nextTri==None)):
                curry=3
            elif(((p.adjTris[0].nextTri==None)and(p.adjTris[1].nextTri==None))or((p.adjTris[1].nextTri==None)and(p.adjTris[2].nextTri==None))or((p.adjTris[0].nextTri==None)and(p.adjTris[2].nextTri==None))):
                curry=2
            else:
                curry=1
            if (curry<pasty):
                first3=p
                pasty=curry
    if(first1!=None):
        return first1
    elif(first2!=None):
        return first2
    elif(first3!=None):
        return first3
    else:
        return other

#pickNext() is a function that chooses the next move in the build process of making a triangle strip. The function uses
#two levels of checking to make the decision of where to travel to next. By checking the adjacent of the adjacent to find
#the lowest number of adjacent triangles two moves ahead, then move one position and run the check again. This makes the
# program function in a less greedy manor and more effectively create a path that follows the lowest number of adjacent
# triangles.
def pickNext(p, length): #length is length of adj
    #initalize place holders to arbitrary values and the returning variable to none
    x=None
    x1=100
    x2=100
    x3=100
    #For triangles with 3 adjacent triangles, check adjacent triangles 0, 1, and 2 for their corresponding adjacent
    # triangles to determine if they have 1, 2 or 3 adjacent triangles that are able to be added to the strip. This will
    # set the values of x1,x2, and x3 to the corresponding sizes which are then later compared.
    if (length==3):
        tri0=p.adjTris[0]
        if (tri0.nextTri==None):
            if ((len(tri0.adjTris)==1)and(tri0.adjTris[0].nextTri==None)):
                x1=1
            elif((len(tri0.adjTris)==2)and((tri0.adjTris[0].nextTri==None)or(tri0.adjTris[1].nextTri==None))):
                if ((tri0.adjTris[0].nextTri==None)and(tri0.adjTris[1].nextTri==None)):
                    x1=2
                else:
                    x1=1
            elif((len(tri0.adjTris)==3)and((tri0.adjTris[0].nextTri==None)or(tri0.adjTris[1].nextTri==None)or(tri0.adjTris[2].nextTri==None))):
                if ((tri0.adjTris[0].nextTri==None)and(tri0.adjTris[1].nextTri==None)and(tri0.adjTris[2].nextTri==None)):
                    x1=3
                elif(((tri0.adjTris[0].nextTri==None)and(tri0.adjTris[1].nextTri==None))or((tri0.adjTris[1].nextTri==None)and(tri0.adjTris[2].nextTri==None))or((tri0.adjTris[2].nextTri==None)and(tri0.adjTris[0].nextTri==None))):
                    x1=2
                else:
                    x1=1

        tri1=p.adjTris[1]
        if (tri1.nextTri==None):
            if ((len(tri1.adjTris)==1)and(tri1.adjTris[0].nextTri==None)):
                x2=1
            elif((len(tri1.adjTris)==2)and((tri1.adjTris[0].nextTri==None)or(tri1.adjTris[1].nextTri==None))):
                if ((tri1.adjTris[0].nextTri==None)and(tri1.adjTris[1].nextTri==None)):
                    x2=2
                else:
                    x2=1
            elif((len(tri1.adjTris)==3)and((tri1.adjTris[0].nextTri==None)or(tri1.adjTris[1].nextTri==None)or(tri1.adjTris[2].nextTri==None))):
                if ((tri1.adjTris[0].nextTri==None)and(tri1.adjTris[1].nextTri==None)and(tri1.adjTris[2].nextTri==None)):
                    x2=3
                elif(((tri1.adjTris[0].nextTri==None)and(tri1.adjTris[1].nextTri==None))or((tri1.adjTris[1].nextTri==None)and(tri1.adjTris[2].nextTri==None))or((tri1.adjTris[2].nextTri==None)and(tri1.adjTris[0].nextTri==None))):
                    x2=2
                else:
                    x2=1

        tri2=p.adjTris[2]
        if (tri2.nextTri==None):
            if ((len(tri2.adjTris)==1)and(tri2.adjTris[0].nextTri==None)):
                x3=1
            elif((len(tri2.adjTris)==2)and((tri2.adjTris[0].nextTri==None)or(tri2.adjTris[1].nextTri==None))):
                if ((tri2.adjTris[0].nextTri==None)and(tri2.adjTris[1].nextTri==None)):
                    x3=2
                else:
                    x3=1
            elif((len(tri2.adjTris)==3)and((tri2.adjTris[0].nextTri==None)or(tri2.adjTris[1].nextTri==None)or(tri2.adjTris[2].nextTri==None))):
                if ((tri2.adjTris[0].nextTri==None)and(tri2.adjTris[1].nextTri==None)and(tri2.adjTris[2].nextTri==None)):
                    x3=3
                elif(((tri2.adjTris[0].nextTri==None)and(tri2.adjTris[1].nextTri==None))or((tri2.adjTris[1].nextTri==None)and(tri2.adjTris[2].nextTri==None))or((tri2.adjTris[2].nextTri==None)and(tri2.adjTris[0].nextTri==None))):
                    x3=2
                else:
                    x3=1

        #This is the comparison determines what triangle to choose next, if there is any equality then arbitrarily pick between the two
        if (x1==x2==x3):
            x=0
        elif (x1==x2 and x1<x3):
            x=0
        elif(x1==x3 and x1<x2):
            x=0
        elif (x2==x3 and x2<x1):
            x=1
        elif (x1<x2 and x1<x3):
            x=0
        elif(x2<x1 and x2<x3):
            x=1
        elif(x3<x1 and x3<x2):
            x=2
    #For triangles with 2 adjacent triangles, check adjacent triangles 0 and 1 for their corresponding adjacent
    # triangles to determine if they have 1, 2 or 3 adjacent triangles that are able to be added to the strip.This will
    # set the values of x1 and x2 to the corresponding sizes which are then later compared.
    elif(length==2):
        tri0 = p.adjTris[0]
        tri1 = p.adjTris[1]
        if (tri0.nextTri==None):
            if ((len(tri0.adjTris)==1)and(tri0.adjTris[0].nextTri==None)):
                x1=1
            elif((len(tri0.adjTris)==2)and((tri0.adjTris[0].nextTri==None)or(tri0.adjTris[1].nextTri==None))):
                if ((tri0.adjTris[0].nextTri==None)and(tri0.adjTris[1].nextTri==None)):
                    x1=2
                else:
                    x1=1
            elif ((len(tri0.adjTris) == 3) and ((tri0.adjTris[0].nextTri == None) or (tri0.adjTris[1].nextTri == None) or (tri0.adjTris[2].nextTri == None))):
                if ((tri0.adjTris[0].nextTri == None) and (tri0.adjTris[1].nextTri == None) and (tri0.adjTris[2].nextTri == None)):
                    x1 = 3
                elif (((tri0.adjTris[0].nextTri == None) and (tri0.adjTris[1].nextTri == None)) or ((tri0.adjTris[1].nextTri == None) and (tri0.adjTris[2].nextTri == None)) or ((tri0.adjTris[2].nextTri == None) and (tri0.adjTris[0].nextTri == None))):
                    x1 = 2
                else:
                    x1 = 1

        if (tri1.nextTri==None):
            if ((len(tri1.adjTris)==1)and(tri1.adjTris[0].nextTri==None)):
                x2=1
            elif((len(tri1.adjTris)==2)and((tri1.adjTris[0].nextTri==None)or(tri1.adjTris[1].nextTri==None))):
                if ((tri1.adjTris[0].nextTri==None)and(tri1.adjTris[1].nextTri==None)):
                    x2=2
                else:
                    x2=1
            elif ((len(tri1.adjTris) == 3) and ((tri1.adjTris[0].nextTri == None) or (tri1.adjTris[1].nextTri == None) or (tri1.adjTris[2].nextTri == None))):
                if ((tri1.adjTris[0].nextTri == None) and (tri1.adjTris[1].nextTri == None) and (tri1.adjTris[2].nextTri == None)):
                    x2 = 3
                elif (((tri1.adjTris[0].nextTri == None) and (tri1.adjTris[1].nextTri == None)) or ((tri1.adjTris[1].nextTri == None) and (tri1.adjTris[2].nextTri == None)) or ((tri1.adjTris[2].nextTri == None) and (tri1.adjTris[0].nextTri == None))):
                    x2 = 2
                else:
                    x2 = 1
        # This is the comparison determines what triangle to choose next, if there is any equality then arbitrarily pick between the two
        if (x1 == x2 == x3):
            x = 0
        elif (x1 == x2 and x1 < x3):
            x = 0
        elif (x1 == x3 and x1 < x2):
            x = 0
        elif (x2 == x3 and x2 < x1):
            x = 1
        elif (x1 < x2 and x1 < x3):
            x = 0
        elif (x2 < x1 and x2 < x3):
            x = 1
        elif (x3 < x1 and x3 < x2):
            x = 2
    return x #return the index in adjTris of the ideal next triangle in the strip


def buildTristrips( triangles ):

    count = 0

    # [YOUR CODE HERE]
    #
    # Increment 'count' every time you *start* a new triStrip.

    copyTri=triangles.copy() #copy list of original triangle list

    while 1:
        p=findFirst(copyTri) #call findFirst function with copy list to find optimum starting point for strip
        if (p==None): #if findFirst returns None then all triangles are in a strip and program exits loop
             break
        wait=1
        while wait!=0:

            sys.stderr.write('Press "p" to proceed ')
            sys.stderr.flush()

            lastKey = None
            while lastKey != 80:  # wait for 'p'
                glfw.wait_events()
                display()
                wait = 0

            sys.stderr.write('\r                     \r')
            sys.stderr.flush()
        count = count + 1 #increment count before starting new strip
        while 1:
            length=len(p.adjTris) #get number of triangles adjacent to current triangle

            #determine if all adjacent triangles are already in a strip. If so then break out of loop and start new strip
            if ((length==3)and((p.adjTris[0].nextTri!=None)and(p.adjTris[1].nextTri!=None)and(p.adjTris[2].nextTri!=None))):
                break
            elif ((length==2)and((p.adjTris[0].nextTri!=None)and(p.adjTris[1].nextTri!=None))):
                break
            elif ((length==1)and((p.adjTris[0].nextTri!=None))):
                break

            #if there is only one adjacent triangle then set nextTri and prevTri accordingly
            if (length==1):
                p.nextTri=p.adjTris[0]
                p.nextTri.prevTri=p
                p=p.nextTri
                copyTri.remove(p.prevTri) #remove triangles from copy list as they are added to a strip

            #if there is more than one adjacent triangle then call pickNext function to find best choice for next
            # triangle in strip
            else:
                x=pickNext(p, length)
                p.nextTri=p.adjTris[x]
                p.nextTri.prevTri=p
                p=p.nextTri
                copyTri.remove(p.prevTri) #remove triangles from copy list as they are added to a strip

    print( 'Generated %d tristrips' % count )


# ================================================================
# ================================================================
# ================================================================



# Set up the display and draw the current image

windowLeft   = None
windowRight  = None
windowTop    = None
windowBottom = None

def display( wait=False ):

    global lastKey, windowLeft, windowRight, windowBottom, windowTop
    
    # Handle any events that have occurred

    glfw.poll_events()

    # Set up window

    glClearColor( 1,1,1,0 )
    glClear( GL_COLOR_BUFFER_BIT )
    glPolygonMode( GL_FRONT_AND_BACK, GL_FILL )

    glMatrixMode( GL_PROJECTION )
    glLoadIdentity()

    glMatrixMode( GL_MODELVIEW )
    glLoadIdentity()

    if maxX-minX > maxY-minY: # wider point spread in x direction
        windowLeft = -0.1*(maxX-minX)+minX
        windowRight = 1.1*(maxX-minX)+minX
        windowBottom = windowLeft
        windowTop    = windowRight
    else: # wider point spread in y direction
        windowTop    = -0.1*(maxY-minY)+minY
        windowBottom = 1.1*(maxY-minY)+minY
        windowLeft   = windowBottom
        windowRight  = windowTop

    glOrtho( windowLeft, windowRight, windowBottom, windowTop, 0, 1 )

    # Draw triangles

    for tri in allTriangles:
        tri.draw()

    # Draw pointers.  Do this *after* the triangles (above) so that the
    # triangle drawing doesn't overlay the pointers.

    for tri in allTriangles:
        tri.drawPointers()

    # Show window

    glfw.swap_buffers( window )

    # Maybe wait until the user presses 'p' to proceed
    
    if wait:

        sys.stderr.write( 'Press "p" to proceed ' )
        sys.stderr.flush()

        lastKey = None
        while lastKey != 80: # wait for 'p'
            glfw.wait_events()
            display()

        sys.stderr.write( '\r                     \r' )
        sys.stderr.flush()


    

# Handle keyboard input

def keyCallback( window, key, scancode, action, mods ):

    global lastKey, showForwardLinks
    
    if action == glfw.PRESS:
    
        if key == glfw.KEY_ESCAPE: # quit upon ESC
            sys.exit(0)
        elif key == ord('F'): # toggle forward/backward link display
            showForwardLinks = not showForwardLinks
        else:
            lastKey = key



# Handle window reshape


def windowReshapeCallback( window, newWidth, newHeight ):

    global windowWidth, windowHeight

    windowWidth  = newWidth
    windowHeight = newHeight



# Handle mouse click/release

def mouseButtonCallback( window, btn, action, keyModifiers ):

    if action == glfw.PRESS:

        # Find point under mouse

        x,y = glfw.get_cursor_pos( window ) # mouse position

        wx = (x-0)/float(windowWidth)  * (windowRight-windowLeft) + windowLeft
        wy = (windowHeight-y)/float(windowHeight) * (windowTop-windowBottom) + windowBottom

        selectedTri = None
        for tri in allTriangles:
            if tri.containsPoint( [wx, wy] ):
                selectedTri = tri
                break

        # print triangle, toggle its highlight1, and toggle the highlight2s of its adjacent triangles

        if selectedTri:
            selectedTri.highlight1 = not selectedTri.highlight1
            print( '%s with adjacent %s' % (selectedTri, repr(selectedTri.adjTris)) )
            for t in selectedTri.adjTris:
                t.highlight2 = not t.highlight2


# Read triangles from a file

def readTriangles( f ):

    global allVerts
    
    errorsFound = False
    
    lines = f.readlines()

    # Read the vertices
    
    numVerts = int( lines[0] )
    allVerts = [ [float(c) for c in line.split()] for line in lines[1:numVerts+1] ]

    # Check that the vertices are valid

    for l,v in enumerate(allVerts):
        if len(v) != 2:
            print( 'Line %d: vertex does not have two coordinates.' % (l+2) )
            errorsFound = True

    # Read the triangles

    numTris = int( lines[numVerts+1] )
    triVerts =  [ [int(v) for v in line.split()] for line in lines[numVerts+2:] ]

    # Check that the triangle vertices are valid

    for l,tvs in enumerate(triVerts):
        if len(tvs) != 3:
            print( 'Line %d: triangle does not have three vertices.' % (l+2+numVerts) )
            errorsFound = True
        else:
            for v in tvs:
                if v < 0 or v >= numVerts:
                    print( 'Line %d: Vertex index is not in range [0,%d].' % (l+2+numVerts,numVerts-1) )
                    errorsFound = True

    # Build triangles

    tris = []

    for tvs in triVerts:
        theseVerts = tvs
        if turn( allVerts[tvs[0]], allVerts[tvs[1]], allVerts[tvs[2]] ) != COLLINEAR:
          tris.append( Triangle( tvs ) ) # (don't include degenerate triangles)

    # For each triangle, find and record its adjacent triangles
    #
    # This would normally take O(n^2) time if done by brute force, so
    # we'll exploit Python's hashed dictionary keys.

    if False:

        for tri in tris: # brute force
            adjTris = []
            for i in range(3):
                v0 = tri.verts[i % 3]
                v1 = tri.verts[(i+1) % 3]
                for tri2 in tris:
                    for j in range(3):
                        if v1 == tri2.verts[j % 3] and v0 == tri2.verts[(j+1) % 3]:
                            adjTris.append( tri2 )
                    if len(adjTris) == 3:
                        break
            tri.adjTris = adjTris

    else: # hashing
      
        edges = {}

        for tri in tris:
            for i in range(3):
                v0 = tri.verts[i % 3]
                v1 = tri.verts[(i+1) % 3]
                key = '%f-%f' % (v0,v1)
                edges[key] = tri

        for tri in tris:
            adjTris = []
            for i in range(3):
                v1 = tri.verts[i % 3] # find a reversed edge of an adjacent triangle
                v0 = tri.verts[(i+1) % 3]
                key = '%f-%f' % (v0,v1)
                if key in edges:
                    adjTris.append( edges[key] )
                if len(adjTris) == 3:
                    break
            tri.adjTris = adjTris

    print( 'Read %d points and %d triangles' % (numVerts,numTris) )

    if errorsFound:
        return []
    else:
        return tris

        
    
# Initialize GLFW and run the main event loop

def main():

    global window, allTriangles, minX, maxX, minY, maxY, r
    
    # Check command-line args

    if len(sys.argv) < 2:
        print( 'Usage: %s filename' % sys.argv[0] )
        sys.exit(1)

    args = sys.argv[1:]
    while len(args) > 1:
        # if args[0] == '-x':
        #     pass
        args = args[1:]

    # Set up window
  
    if not glfw.init():
        print( 'Error: GLFW failed to initialize' )
        sys.exit(1)

    window = glfw.create_window( windowWidth, windowHeight, "Assignment 2", None, None )

    if not window:
        glfw.terminate()
        print( 'Error: GLFW failed to create a window' )
        sys.exit(1)

    glfw.make_context_current( window )
    glfw.swap_interval( 1 )
    glfw.set_key_callback( window, keyCallback )
    glfw.set_window_size_callback( window, windowReshapeCallback )
    glfw.set_mouse_button_callback( window, mouseButtonCallback )

    # Read the triangles.  This also fills in the global 'allVerts'.

    with open( args[0], 'rb' ) as f:
        allTriangles = readTriangles( f )

    if allTriangles == []:
        return

    # Get bounding box of points

    minX = min( p[0] for p in allVerts )
    maxX = max( p[0] for p in allVerts )
    minY = min( p[1] for p in allVerts )
    maxY = max( p[1] for p in allVerts )

    # Adjust point radius in proportion to bounding box
    
    if maxX-minX > maxY-minY:
        r *= maxX-minX
    else:
        r *= maxY-minY

    # Run the code

    # start=time.time()
    #
    buildTristrips( allTriangles )
    #
    # time.sleep(1)
    # end=time.time()
    #
    # print("Runtime is: %.4f"%(end-start))

    # Show result and wait to exit

    display( wait=True )
    
    glfw.destroy_window( window )
    glfw.terminate()
    


if __name__ == '__main__':
    main()
