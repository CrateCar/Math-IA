import math
import random
import turtle

final_positions = [0,0] # calc x, y
end_effector_positions = [6.949,3.895]

angle1 = 0.87#math.atan2(end_effector_positions[1],end_effector_positions[0])#(random.randrange(-100,100)/100) * (2*math.pi)
angle2 = 1.55#1#(random.randrange(-100,100)/100) * (2*math.pi)

L1 = 7.4
L2 = 2.8

J = [
    -L1 * (math.sin(angle1)) - L2 * (math.sin(angle1+angle2)), -L2 * (math.sin(angle1+angle2)),
    L1 * (math.cos(angle1)) + L2 * (math.cos(angle1+angle2)), L2 * (math.cos(angle1+angle2))
]

I = [1,0,0,1]

def calc_positions(initial_pos,a1,a2,l1,l2):
    p1 = dot2by1([math.cos(a1),-math.sin(a1),math.sin(a1),math.cos(a1)],[l1,0])
    x1 = p1[0]+initial_pos[0]
    y1 = p1[1]+initial_pos[1]
    p2 = dot2by1([math.cos(a1+a2),-math.sin(a1+a2),math.sin(a1+a2),math.cos(a1+a2)],[l2,0])
    x2 = x1+p2[0]
    y2 = y1+p2[1]
    return [x2,y2]

def transpose(matrix,rows):
    new_matrix = []

    for i in range(0,rows):
        for j in range(0,len(J)):
            if j % rows == i:
                new_matrix.append(matrix[j])
    
    return new_matrix

def dot2by2(matrix1,matrix2):
    new_matrix = [
        matrix1[0]*matrix2[0]+matrix1[1]*matrix2[2], matrix1[0]*matrix2[1]+matrix1[1]*matrix2[3],
        matrix1[2]*matrix2[0]+matrix1[3]*matrix2[2], matrix1[2]*matrix2[1]+matrix1[3]*matrix2[3]
    ]

    return new_matrix

def dot2by1(matrix1,matrix2):
    new_matrix = [
        matrix1[0]*matrix2[0]+matrix1[1]*matrix2[1],
        matrix1[2]*matrix2[0]+matrix1[3]*matrix2[1]
    ]

    return new_matrix

def multi_matrix(matrix,multiplier):
    new_matrix = []
    for item in matrix:
        new_matrix.append(item*multiplier)
    return new_matrix

def add_matrix(matrix1,matrix2):
    new_matrix = []
    for i in range(0,len(matrix1)):
        new_matrix.append(matrix1[i]+matrix2[i])
    return new_matrix

def invert(matrix):
    a = matrix[0]
    b = matrix[1]
    c = matrix[2]
    d = matrix[3]

    multiplier = 1/((a*d)-(b*c))
    print((multiplier))
    new_matrix = [d,-b,-c,a]

    new_matrix = multi_matrix(matrix,multiplier)

    return new_matrix

def set_jacobian(l1,l2,a1,a2):
    matrix = [
         -l1 * (math.sin(a1)) - l2 * (math.sin(a1+a2)), -l2 * (math.sin(a1+a2)),           
        l1 * (math.cos(a1)) + l2 * (math.cos(a1+a2)), l2 * (math.cos(a1+a2))
    ]
    return matrix

def calc_error(xd,x,yd,y):
    e = [xd-x,yd-y]
    return e

def JacobianInverse(matrix,pos,end_effector):
    lambd = 0.4
    error_matrix = calc_error(end_effector[0],pos[0],end_effector[1],pos[1])
    return dot2by1(invert(add_matrix(dot2by2(transpose(matrix,2),matrix),multi_matrix(I,lambd**2))),dot2by1(transpose(matrix,2),error_matrix)) # multiply by dot2by1 (JT,e)


err_dist = math.sqrt((end_effector_positions[0]-final_positions[0])**2 + (end_effector_positions[1]-final_positions[1])**2)
alpha = 0.6
final_positions = calc_positions((0,0),angle1,angle2,L1,L2)
J = set_jacobian(L1,L2,angle1,angle2)
J_inv = JacobianInverse(J,final_positions,end_effector_positions)


while err_dist >= 0.15:
    final_positions = calc_positions((0,0),angle1,angle2,L1,L2)
    J = set_jacobian(L1,L2,angle1,angle2)
    J_inv = JacobianInverse(J,final_positions,end_effector_positions)
    angle1 = angle1 + alpha * J_inv[0]
    angle2 = angle2 + alpha * J_inv[1]
    err_dist = math.sqrt((end_effector_positions[0]-final_positions[0])**2 + (end_effector_positions[1]-final_positions[1])**2)

print(angle1 % math.pi,angle2 % math.pi)

pen = turtle.pen()

while True:
    pos1 = calc_positions((0,0),angle1,angle2,L1,0)
    pos2 = calc_positions((0,0),angle1,angle2,L1,L2)

    turtle.penup()
    turtle.goto(0,0)
    turtle.pendown()
    turtle.goto(int(pos1[0]*50),int(pos1[1]*50))
    turtle.goto(int(pos2[0]*50),int(pos2[1]*50))

    #end_effector_positions = [random.randrange(50,150)/100,random.randrange(50,150)/100]

    err_dist = 0.5

    turtle.clear()
