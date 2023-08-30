""" Written by Brian Hou for CSE571: Probabilistic Robotics
"""

import numpy as np
import matplotlib.pyplot as plt
#plt.ion()


def minimized_angle(angle):
    """Normalize an angle to [-pi, pi]."""
    while angle < -np.pi:
        angle += 2 * np.pi
    while angle >= np.pi:
        angle -= 2 * np.pi
    return angle

def wrap_to_pi(angle):
    """Normalize an angle to [-pi, pi]."""
    angle = ( angle + np.pi) % (2 * np.pi ) - np.pi

    return angle


def plot_landmark(env):
    """Plot the soccer field, highlighting the currently observed marker."""

    ax = env.get_figure().gca(
        aspect='equal',
        xlim=( -50, 50),
        ylim=( -30, 50)
    )

    for m in range(0, env.landmarks.shape[0]):
        x, y = env.landmarks[m, 0], env.landmarks[m, 1]
        plot_circle(
            ax, (x, y), radius=1,
            facecolor=('w'))

        #plt.annotate(m, xy=(x, y), ha='center', va='center')

def plot_milestones(env, milestones):
    ax = env.get_figure().gca()
    for m in range(0, milestones.shape[0]):
        x, y = milestones[m, 0], milestones[m, 1]
        plot_circle(
            ax, (x, y), radius=1, edgecolor='y', facecolor='y')

def plot_robot(env, x, color, radius=1):
    """Plot the robot on the soccer field."""
    ax = env.get_figure().gca()
    plot_circle(ax, x[:2], radius=radius, facecolor=color)
    theta = np.deg2rad(x[2])
    # robot orientation
    ax.plot(
        [x[0], x[0] + np.cos(theta) * (radius + 10)],
        [x[1], x[1] + np.sin(theta) * (radius + 10)],
        color)

    # observation
    #ax.plot(
    #    [x[0], x[0] + np.cos(x[2] + z[0]) * 100],
    #    [x[1], x[1] + np.sin(x[2] + z[0]) * 100],
    #    'b', linewidth=0.5)


def plot_path(env, states, color, linewidth=1):
    """Plot a path of states."""
    ax = env.get_figure().gca()
    ax.plot(states[:, 0], states[:, 1], color=color, linewidth=linewidth)

def plot_line(env, pos0, pos1, linewidth=1):
    """Plot a path of states."""
    ax = env.get_figure().gca()
    ax.plot((pos0[0], pos1[0]), (pos0[1], pos1[1]), color='red', linewidth=linewidth)


def plot_particles(env, x):
    ax = env.get_figure().gca()
    for i in range(0, x.shape[0]):
        ax.arrow(x[i, 0], x[i, 1], np.cos(x[i, 2]) * 1, np.sin(x[i, 2]) * 1,  color = 'g', head_width=0.2)


def plot_measure(env, xy, radius, edgecolor='k', facecolor='w', **kwargs):
    """Plot a circle."""
    ax = env.get_figure().gca()
    circle = plt.Circle(
        xy,
        radius=radius,
        fill=False,
        edgecolor=edgecolor,
        facecolor=facecolor,
        **kwargs)
    ax.add_patch(circle)

def plot_circle(ax, xy, radius, edgecolor='k', facecolor='w', **kwargs):
    """Plot a circle."""
    circle = plt.Circle(
        xy,
        radius=radius,
        fill=True,
        edgecolor=edgecolor,
        facecolor=facecolor,
        **kwargs)
    ax.add_artist(circle)
