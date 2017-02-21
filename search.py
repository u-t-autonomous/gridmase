import dubins
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from numpy import pi, cos, sin
from math import tan, sqrt, ceil, atan2
from scipy.spatial import ConvexHull

def deg2rad(deg):
    return deg*pi/180

def rad2deg(rad):
    return rad*(180/pi)

def polar_to_rec(r,theta):
    x = r*cos((theta+(pi/2))%(2*pi))
    y = r*sin((theta+(pi/2))%(2*pi))
    return (y,x)

def theta2heading(theta):
    assert (theta <= 2*pi and theta >= 0), "Make sure it's in radians"
    heading = 2*pi - ( (pi/2.) + theta)
    return heading

def translate_point(center,point,b):
    if b:
        translated_point = (point[0]-center[0],point[1]-center[1])
    else:
        translated_point = (point[0]+center[0],point[1]+center[1])
    return translated_point

def get_arch_p(d_w = None, alpha = None, step = deg2rad(90.),extent = None):
    r = 0
    theta = 0
    rs = []
    thetas = []
    
    while r < extent:
        r = d_w/4. + alpha*d_w*theta
        rs.append(r)
        thetas.append(theta % 2*pi)
        theta += step
       
    points = map(polar_to_rec,rs,thetas)
    
    return points,thetas
    
def PolyArea2D(pts):
    lines = np.hstack([pts,np.roll(pts,-1,axis=0)])
    area = 0.5*abs(sum(x1*y2-x2*y1 for x1,y1,x2,y2 in lines))
    return area

def make_convex(ps):
    hull = ConvexHull(ps)
    points = [hull.points[i].tolist() for i in hull.vertices]
    return points

def make_circle(poly):
    hull = make_convex(poly)
    center = find_center(hull)
    distances = map(get_distance,hull,[center]*len(hull))
    return (center, max(distances))
   
def find_center(polygon):
    area = PolyArea2D(polygon)
    imax = len(polygon) - 1

    result_x = 0
    result_y = 0
    
    for i in range(0,imax):
        result_x += (polygon[i][0] + polygon[i+1][0]) * ((polygon[i][0] * polygon[i+1][1]) - (polygon[i+1][0] * polygon[i][1]))
        result_y += (polygon[i][1] + polygon[i+1][1]) * ((polygon[i][0] * polygon[i+1][1]) - (polygon[i+1][0] * polygon[i][1]))
    
    result_x += (polygon[imax][0] + polygon[0][0]) * ((polygon[imax][0] * polygon[0][1]) - (polygon[0][0] * polygon[imax][1]))
    result_y += (polygon[imax][1] + polygon[0][1]) * ((polygon[imax][0] * polygon[0][1]) - (polygon[0][0] * polygon[imax][1]))
    result_x /= (area * 6.0)
    result_y /= (area * 6.0)

    return (result_x, result_y)

def rotate_hull(hull,angle):
    center = find_center(hull)  
    return map(rotate_point,hull,[center]*len(hull),[angle]*len(hull))

def rotate_point(point,center,angle):
    point_t = translate_point(center,point,True)
    x_r = point_t[0]*cos(angle) - point_t[1]*sin(angle)
    y_r = point_t[1]*cos(angle) - point_t[0]*sin(angle)
    point_r = (x_r,y_r)
    point = translate_point(center,point,False)
    return point

def expand_axis(ax, scale, name):
    getter = getattr(ax, 'get_' + name)
    setter = getattr(ax, 'set_' + name)
    a, b = getter()
    mid = (a+b)/2.0
    diff = (b - mid)
    setter(mid - scale*diff, mid + scale*diff)

def expand_plot(ax, scale = 1.1):
    expand_axis(ax, scale, 'xlim')
    expand_axis(ax, scale, 'ylim')

# this function returns the dubins path of two configurations
def get_path(vs=None, ve=None, turning_radius=None, step_size=None):
    path,_ = dubins.path_sample(vs, ve, turning_radius, step_size)
    return path

def add_ang(p,x):
    v = (p[0],p[1],x)
    return v

# this function returns the starting configuration of a point inspection
def get_vs(p,aoa,d_l=None):
    return (p[0] + d_l*cos(aoa),p[1] + d_l*sin(aoa),aoa+pi)

# this function returns the ending configuration of a point inspection
def get_ve(p,aoa,d_t=None):   
    return (p[0] + d_t*cos(aoa),p[1] + d_t*sin(aoa),aoa+pi)

def dl(h_alt,theta_g,vfov):
    return h_alt/(tan(theta_g - vfov/2))

def dc(h_alt,theta_g):
    return h_alt/tan(theta_g)
    
def dt(h_alt,theta_g,vfov):
    return h_alt/(tan(theta_g + vfov/2))

def dw(h_alt, theta_g, hfov):
    return 2*(h_alt/cos(theta_g))*tan(hfov/2)

def discretize(xc = None, xr = None, disc = 10):
    set_of_angles = []
    
    if xc != None:
        if xr != None:
            upper_x = xc + (xr/2.)
            bottom_x = xc - (xr/2.)
        else:
            upper_x = xc
            bottom_x = xc
    else:
        upper_x = 360
        bottom_x = 0
        
    step = ( upper_x - bottom_x )/disc
    set_of_angles = [bottom_x+(i*step) for i in range(disc+1)]
    set_of_angles = list(set(set_of_angles))
    return set_of_angles

def diffsq(p1,p2):
    return (p1-p2)**2

def get_distance(p1,p2):
    return sqrt(sum(map(diffsq,p1,p2)))

def calc_dist(pre_ve,vs):
    total_cost = 0
    path,_ = dubins.path_sample(pre_ve, vs, 1, .1)
    total_cost = sum(map(get_distance,path[:-1],path[1:]))
    return total_cost

def aoa(pre_ve, p, xc = None, xr = None, d_l = None, disc = 10):
    set_of_a = discretize(xc,xr,disc)

    set_of_vs = map(get_vs,[p]*len(set_of_a),set_of_a, [d_l]*len(set_of_a))
    distance = map(calc_dist,[pre_ve]*len(set_of_vs),set_of_vs)
    return set_of_a[distance.index(min(distance))]

def plot_dubins_path(qs):
    qs = np.array(qs)
    xs = qs[:, 1]
    ys = qs[:, 0]
    us = xs + cos(qs[:, 2])
    vs = ys + sin(qs[:, 2])
    plt.plot(xs, ys, 'b-')
    plt.plot(xs, ys, 'r.')
    for i in xrange(qs.shape[0]):
        plt.plot([xs[i], us[i]], [ys[i], vs[i]],'r-')
    ax = plt.gca()
    expand_plot(ax)
    ax.set_aspect('equal')  

def area_triangle(a,b,c):
    s = (a + b + c)/2.
    return sqrt(s*(s - a)*(s - b)*(s - c))

def smooth_line(line,n=10):
    new_line = line
    while len(new_line) > n:
        p1 = new_line[:-2]
        p2 = new_line[1:-1]
        p3 = new_line[2:]
        a = map(get_distance,p1,p2)
        b = map(get_distance,p1,p3)
        c = map(get_distance,p2,p3)
        areas = map(area_triangle,a,b,c)
        new_line = [x for x in new_line if x != new_line[areas.index(min(areas))+1]]
    return new_line

def point_search(pre_ve,p,xc=None,xr=None,s=None,d_t=None, d_l = None, turning_radius = None, step_size = 0.5):
    vs = get_vs(p,aoa(pre_ve,p,xc,xr,d_l),d_l = s)
    ve = get_ve(p,aoa(pre_ve,p,xc,xr,d_l),d_t)
    path_task1 = get_path(vs = pre_ve, ve = vs, turning_radius = turning_radius, step_size = step_size)
    path_task2 = get_path(vs = vs, ve = ve, turning_radius = turning_radius, step_size = step_size)
    return path_task1+path_task2

def line_search(pre_ve,line,angles,smoothing=10, b = False, turning_radius = None, step_size = 0.5):
        
    line = smooth_line(line,smoothing) 
    seg_a = line[:-1]
    seg_b = line

    if not b:
        first = seg_a[0]
        last = seg_b[-1]
        angle = atan2((last[1]-first[1]),(last[0]-first[0]))
        if angle >= 0:
            angles = angle  #make better angles
        else:
            angles = (2*pi) - angle

    if isinstance(angles,list):
        seg_a_angles = angles[:-1]    
        seg_b_angles = angles
        seg_vs = map(add_ang,seg_a,seg_a_angles)
        seg_ve = map(add_ang,seg_b,seg_b_angles)
        seg_vs.insert(0,pre_ve)
    else:
        seg_vs = map(add_ang,seg_a,[angles]*len(seg_a))
        seg_ve = map(add_ang,seg_b,[angles]*len(seg_b)) 
        seg_vs.insert(0,pre_ve)

    
    paths = map(get_path,seg_vs,seg_ve,[turning_radius]*len(line),[step_size]*len(line))
    path = [item for sublist in paths for item in sublist]
    return path

def spiral_search(pre_ve,p,d_w,d_l,alpha,extent, step = deg2rad(36),turning_radius = None, step_size = 0.5):
    points, thetas = get_arch_p(d_w = d_w, alpha = alpha, extent = extent, step = step)
    points = map(translate_point,[p]*len(points),points,[False]*len(points))
    vs = (p[0]+(d_w/4.),p[1]+d_l,deg2rad(270))
    headings = map(theta2heading,thetas)
    v = map(add_ang,points,headings)
    v.insert(0,vs)
    v.insert(0,pre_ve)
    vs = v[:-1]
    ve = v[1:]

    assert (len(vs) < 200), "The length of points %r is probably too high!" % len(vs)
    paths = map(get_path,vs,ve,[turning_radius]*len(vs),[step_size]*len(vs))
    path = [item for sublist in paths for item in sublist]
    return path
    
#add option for point inspect
#add angle rotation

def area_search(pre_ve=None,poly=None,angle=None,alpha=None,d_w=None,p=None,xc=None,xr=None,s=None,d_t=0,d_l=0,turning_radius=None, step_size=None):
    y = []
    v = []
    
    if isinstance(p,tuple):
        path = point_search(pre_ve,p,xc,xr,s,d_t)
        pre_ve = path[-1]
    else:
        path = []

    center, radius = make_circle(poly)
    far_w = center[1] - radius
    far_e = center[1] + radius
    y.append(far_w + ((alpha*d_w)/2.))
    i = 0
    
    while (far_e - (y[i-1] + alpha*d_w)) > ((alpha*d_w)/2):
        if i != 0:
            y.append(y[i-1] + alpha*d_w)

        inter = sqrt(radius**2 - (y[i] - center[1])**2)
        far_n = inter + center[0]
        far_s = -inter + center[0]
        
        if i % 2 == 0:
            v.append((far_s - d_l,y[i],0))
            v.append((far_n - d_t,y[i],0))
        else:
            v.append((far_n + d_l,y[i],pi))
            v.append((far_s + d_t,y[i],pi))
        i += 1
        
    seg_vs = v[:-1]
    seg_ve = v
    seg_vs.insert(0,pre_ve)
    
    paths = map(get_path,seg_vs,seg_ve,[turning_radius]*len(v),[step_size]*len(v))
    path = [item for sublist in paths for item in sublist]
    return path

def sector_search(p, extent, alpha):
    X = []
    v = []
    X.append(deg2rad(90))
    N = ceil(2*pi*extent)/(alpha*d_w)
    v.append((p[0] - (2*turning_radius),p[1] - extent - d_l,deg2rad(270)))
    v.append((p[0],p[1] - extent - d_l,X[0]))
    v.append((p[0],p[1] + extent - d_l,X[0]))
    for i in range(1,N):
        X.append = X[i-1] - (2*pi*((alpha*d_w/(2*pi*extent) - 1/2)))
        v.append((p,X[i]-pi))


if __name__ == "__main__":

    h_alt = 1
    theta_g = deg2rad(55)
    vfov = deg2rad(55)

    d_l = 500
    d_t = 200
    d_w = 100

    turning_radius = 1.0
    step_size = .2

    xc = deg2rad(180)
    xr = deg2rad(70)
    pre_xe = deg2rad(90) 
    pre_ve = (1,2,pre_xe)

    p = (5,4)

    p2 = (7,2)

    s = d_l

    path = spiral_search((0,0),d_w = d_w,alpha = 0.9,extent = 1200, d_l = d_l,step = deg2rad(10.))
    plot_dubins_path(path)
    plt.show()