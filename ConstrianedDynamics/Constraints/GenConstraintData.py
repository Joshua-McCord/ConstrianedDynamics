from sympy import *
from sympy.matrices import *
from sympy.parsing import *

FOLDER_DIR = 'C:/Users/Josh/source/repos/ConstrianedDynamics/ConstrianedDynamics/Constraints/'

def parseFile(fileName):
    lst = []
    symbols = set()
    with open(fileName) as f:
        lines = f.readlines()
    for ln in lines:
        eq = parse_expr(ln.strip())
        lst.append(eq)

        for sym in eq.free_symbols:
            symbols.add(sym)
    return (lst, len(symbols) // 2)


def writeCDot(Cdot):
    with open(FOLDER_DIR + 'constraints_dot.txt', 'w') as f:
        for item in Cdot:
            f.write("%s\n" % item)

def writeForces(n_particles):
    with open(FOLDER_DIR + 'forces.txt', 'w') as f:
        for i in range(n_particles):
            f.write('0,-10\n')

def writeMasses(n_particles):
    mat = eye(n_particles * 2)
    with open(FOLDER_DIR + 'masses.txt', 'w') as f:
        for i in range(n_particles * 2):
            for j in range(n_particles * 2):
                print(mat[i,j], end = '')
                f.write("%i" % mat[i,j])
                if j != (n_particles * 2) - 1:
                    f.write(",")

            f.write("\n")

(C, n_particles) = parseFile(FOLDER_DIR + 'constraints.txt')

Cdot = []
for eq in C:
    sum = 0 
    for i in range(1, n_particles+1):
        sum += (diff(eq, f'x{i}') * Symbol(f'vx{i}') + diff(eq, f'y{i}') * Symbol(f'vy{i}'))

    Cdot.append(sum)


writeCDot(Cdot)
writeForces(n_particles)
writeMasses(n_particles)

print('Hello, World')

#0.5 * ((x2 - x3)*(x2 - x3) + (y2 - y3)*(y2 - y3)) - 0.125
#
#0.6,-0.2
#
#0.0,0.0