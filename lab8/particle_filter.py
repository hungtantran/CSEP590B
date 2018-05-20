from grid import *
from particle import Particle
from utils import *
from setting import *
import random
import scipy.stats
import numpy.random


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
                heading_sigma=ODOM_HEAD_SIGMA,
                trans_sigma=ODOM_TRANS_SIGMA
            )
        )
    return particles

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
    xy_dst = scipy.stats.norm(0, MARKER_TRANS_SIGMA)
    rt_dst = scipy.stats.norm(0, MARKER_ROT_SIGMA)

    skip = 0
    not_skip = 0
    if len(measured_marker_list) == 0:
        return particles
    else:
        for particle in particles:
            if not grid.is_in(particle.x, particle.y):
                skip += 1
                weights.append(0)
                continue
            particle_marker_list = particle.read_markers(grid)

            weight = 1.0 / len(particles)
            if len(particle_marker_list) == 0:
                weight = 0
            elif len(particle_marker_list) == 1 and len(measured_marker_list) == 1:
                weight = (xy_dst.pdf(particle_marker_list[0][0] - measured_marker_list[0][0]) *
                          xy_dst.pdf(particle_marker_list[0][1] - measured_marker_list[0][1]) *
                          rt_dst.pdf(particle_marker_list[0][2] - measured_marker_list[0][2]))
            elif len(particle_marker_list) == 1 and len(measured_marker_list) == 2:
                weight = (xy_dst.pdf(particle_marker_list[0][0] - measured_marker_list[0][0]) *
                          xy_dst.pdf(particle_marker_list[0][1] - measured_marker_list[0][1]) *
                          rt_dst.pdf(particle_marker_list[0][2] - measured_marker_list[0][2])
                          *
                          xy_dst.pdf(particle_marker_list[0][0] - measured_marker_list[1][0]) *
                          xy_dst.pdf(particle_marker_list[0][1] - measured_marker_list[1][1]) *
                          rt_dst.pdf(particle_marker_list[0][2] - measured_marker_list[1][2]))
            elif len(particle_marker_list) == 2 and len(measured_marker_list) == 1:
                weight = (xy_dst.pdf(particle_marker_list[0][0] - measured_marker_list[0][0]) *
                          xy_dst.pdf(particle_marker_list[0][1] - measured_marker_list[0][1]) *
                          rt_dst.pdf(particle_marker_list[0][2] - measured_marker_list[0][2])
                          +
                          xy_dst.pdf(particle_marker_list[1][0] - measured_marker_list[0][0]) *
                          xy_dst.pdf(particle_marker_list[1][1] - measured_marker_list[0][1]) *
                          rt_dst.pdf(particle_marker_list[1][2] - measured_marker_list[0][2]))
            elif len(particle_marker_list) == 2 and len(measured_marker_list) == 2:
                weight = ((xy_dst.pdf(particle_marker_list[0][0] - measured_marker_list[0][0]) *
                          xy_dst.pdf(particle_marker_list[0][1] - measured_marker_list[0][1]) *
                          rt_dst.pdf(particle_marker_list[0][2] - measured_marker_list[0][2])
                          +
                          xy_dst.pdf(particle_marker_list[1][0] - measured_marker_list[0][0]) *
                          xy_dst.pdf(particle_marker_list[1][1] - measured_marker_list[0][1]) *
                          rt_dst.pdf(particle_marker_list[1][2] - measured_marker_list[0][2]))
                          *
                          (xy_dst.pdf(particle_marker_list[0][0] - measured_marker_list[1][0]) *
                          xy_dst.pdf(particle_marker_list[0][1] - measured_marker_list[1][1]) *
                          rt_dst.pdf(particle_marker_list[0][2] - measured_marker_list[1][2])
                          +
                          xy_dst.pdf(particle_marker_list[1][0] - measured_marker_list[1][0]) *
                          xy_dst.pdf(particle_marker_list[1][1] - measured_marker_list[1][1]) *
                          rt_dst.pdf(particle_marker_list[1][2] - measured_marker_list[1][2])))

            not_skip += 1
            weights.append(weight)
            total_weight += weight
    print("%d %d" % (skip, not_skip))
    if total_weight == 0:
        weights = [1.0 / len(particles) for _ in particles]
    else:
        weights = [float(weight)/total_weight for weight in weights]
    particles = numpy.random.choice(particles, size=len(particles), replace=True, p=weights)
    measurement_particles = []
    for particle in particles:
        measurement_particles.append(Particle(particle.x, particle.y, particle.h))
    return measurement_particles