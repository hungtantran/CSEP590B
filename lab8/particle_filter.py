from grid import *
from particle import Particle
from utils import *
from setting import *
import random
import scipy.stats


def motion_update(particles, odom):
    """ Particle filter motion update

        Arguments: 
        particles -- input list of particle represents belief p(x_{t-1} | u_{t-1})
                before motion update
        odom -- odometry to move (dx, dy, dh) in *robot local frame*

        Returns: the list of particles represents belief \tilde{p}(x_{t} | u_{t})
                after motion update
    """
    for particle in particles:
        particle.move(
            add_odometry_noise(
                odom,
                heading_sigma=0,
                trans_sigma=0
            )
        )
    return particles

def within(a, b, d):
    if a + d < b:
        return False
    if a - d > b:
        return False
    return True

# ------------------------------------------------------------------------
def measurement_update(particles, measured_marker_list, grid):
    """ Particle filter measurement update

        Arguments: 
        particles -- input list of particle represents belief \tilde{p}(x_{t} | u_{t})
                before meansurement update (but after motion update)

        measured_marker_list -- robot detected marker list, each marker has format:
                measured_marker_list[i] = (rx, ry, rh)
                rx -- marker's relative X coordinate in robot's frame
                ry -- marker's relative Y coordinate in robot's frame
                rh -- marker's relative heading in robot's frame, in degree

                * Note that the robot can only see markers which is in its camera field of view,
                which is defined by ROBOT_CAMERA_FOV_DEG in setting.py
				* Note that the robot can see mutliple markers at once, and may not see any one

        grid -- grid world map, which contains the marker information, 
                see grid.py and CozGrid for definition
                Can be used to evaluate particles

        Returns: the list of particles represents belief p(x_{t} | u_{t})
                after measurement update
    """
    weights = []
    total_weight = 0
    """
        Calculating weights
    """
    dst = scipy.stats.multivariate_normal(
        mean = [0, 0, 0],
        cov = [ [MARKER_TRANS_SIGMA, 0, 0], [0, MARKER_TRANS_SIGMA, 0], [0, 0, MARKER_ROT_SIGMA] ]
    )
    xy_dst = scipy.stats.norm(0, MARKER_TRANS_SIGMA)
    rt_dst = scipy.stats.norm(0, MARKER_ROT_SIGMA)

    for particle in particles:
        if not grid.is_in(particle.x, particle.y):
            weights.append(0)
            continue

        """predicted_marker_list = particle.read_markers(grid)
        if not within(len(predicted_marker_list), len(measured_marker_list), 1):
            weights.append(0.1)
            total_weight += 0.1
            #print("Remove 1 %d %d" % (len(predicted_marker_list), len(measured_marker_list)))
            continue

        weight = 0.1
        if len(predicted_marker_list) == len(measured_marker_list):
            weight += 0.1"""

        marker_list = []
        for i, marker in enumerate(grid.markers):
            m_x, m_y, m_h = parse_marker_info(marker[0], marker[1], marker[2])
            mr_x, mr_y = rotate_point(m_x - particle.x, m_y - particle.y, -particle.h)
            mr_h = diff_heading_deg(m_h, particle.h)
            marker_list.append((mr_x, mr_y, mr_h, i))

        #print("%s\n %s\n" % (measured_marker_list, marker_list))
        weight = 0
        max_fit_markers = []
        max_probs = []
        for measured_marker in measured_marker_list:
            max_prob = -1
            max_fit_marker = None
            for marker in marker_list:
                prob = dst.pdf([
                    measured_marker[0] - marker[0],
                    measured_marker[1] - marker[1],
                    measured_marker[2] - marker[2]
                ])
                if prob > max_prob:
                    max_prob = prob
                    max_fit_marker = marker
            max_fit_markers.append(max_fit_marker)
            max_probs.append(max_prob)

        for prob in max_probs:
            weight += prob
        if len(max_probs) > 0:
            weight = float(weight)/len(max_probs)

        #weight += xy_dst.pdf(measured_marker[0] - marker[0])
        #weight += xy_dst.pdf(measured_marker[1] - marker[1])
        #weight += rot_dst.pdf(measured_marker[2] - marker[2])
        """prob = dst.pdf([
            measured_marker[0] - marker[0],
            measured_marker[1] - marker[1],
            measured_marker[2] - marker[2]
        ])
        weight += prob
        if measured_marker[3] == marker[3]:
            print("%s %s %s" % (measured_marker, marker, prob))"""
        #print(weight)
        """if not within(measured_marker[0], marker[0], MARKER_TRANS_SIGMA * 10):
            #print("Remove 2")
            continue

        if not within(measured_marker[1], marker[1], MARKER_TRANS_SIGMA * 10):
            #print("Remove 3")
            continue

        if not within(measured_marker[2], marker[2], MARKER_ROT_SIGMA * 10):
            #print("Remove 4")
            continue

        found = True

        if not found:
            weight = 0.2
            break"""

        weights.append(weight)
        total_weight += weight
    if total_weight == 0:
        weights = [float(1)/len(weights) for _ in weights]
    else:
        weights = [float(weight)/total_weight for weight in weights]
    print("%s %s " % (total_weight, measured_marker_list))

    """
        Resampling
    """
    measured_particles = []
    while True:
        for index, weight in enumerate(weights):
            # Find enough sample, return            
            if len(measured_particles) == len(particles):
                return measured_particles
            # Sampling
            rand = random.random()
            if rand <= weight:
                measured_particles.append(particles[index])
    # Should never reach here
    return None


