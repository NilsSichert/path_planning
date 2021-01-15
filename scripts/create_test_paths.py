#!/usr/bin/env python
import yaml
import numpy as np
import os
import matplotlib.pyplot as plt


def lemniscate_of_bernoulli(t, a=1.0):
    # https://de.wikipedia.org/wiki/Lemniskate_von_Bernoulli
    # 0 <= t < 2*Pi
    # start point for t=0: far right corner (x at positive max, y = 0)
    # a: distance between origin/middle of 8 and "center" of circle
    # approximately the same "height" of inf-sign
    x = (a * np.sqrt(2) * np.cos(t)) / (np.sin(t)**2 + 1.0)
    y = (a * np.sqrt(2) * np.cos(t) * np.sin(t)) / (np.sin(t)**2 + 1.0)
    return x, y


def lemniscate_of_gerono(t, a=1.0):
    # https://mathworld.wolfram.com/EightCurve.html
    # 0 <= t < 2*pi
    # start point for t=0: origin, going towards top right corner
    # (x and y positive)
    # a: same a as for bernoulli -> same "height"
    x = a * np.sin(t)
    y = a * np.sin(t) * np.cos(t)
    return x, y


def super_ellipse(t, a=1.0, b=3.0, n=4.0):
    # https://en.wikipedia.org/wiki/Superellipse
    # 0 <= t < 2*pi
    x = np.abs(np.cos(t))**(2.0 / n) * a * np.sign(np.cos(t))
    y = np.abs(np.sin(t))**(2.0 / n) * b * np.sign(np.sin(t))
    return x, y


def circle(t, r=1.0):
    x = r * np.cos(t)
    y = r * np.sin(t)
    return x, y


def get_yaw_angle(x, y):
    # numerical computation of yaw angle using a line through the current and
    # the next waypoint
    # this works for any continous curve
    yaw = [0] * len(x)
    for i in range(len(x) - 1):
        yaw[i] = np.arctan2(y[i + 1] - y[i], x[i + 1] - x[i])
    # assuming that the next waypoint of the last waypoint is the first waypoint
    yaw[-1] = np.arctan2(y[0] - y[-1], x[0] - x[-1])

    return yaw


def scale_and_move(x, y, x_offset=0.7, y_offset=2.0, factor=1.0):
    # x is the shorter side of the tank, check this:
    if np.max(np.abs(x)) > np.max(np.abs(y)):
        # swap axes
        tmp = x
        x = y
        y = tmp

    x *= factor
    y *= factor

    x += x_offset
    y += y_offset

    return x, y


def generate_waypoints(factor=1.0,
                       x_offset=0.7,
                       y_offset=2.0,
                       depth=0.7,
                       number=100):
    # don't want duplicate of first point as last point -> endpoint=False
    t = np.linspace(0, 2.0 * np.pi, num=number, endpoint=False)
    x_raw, y_raw = lemniscate_of_bernoulli(t)
    x, y = scale_and_move(x_raw,
                          y_raw,
                          x_offset=x_offset,
                          y_offset=y_offset,
                          factor=1.0)

    yaw = get_yaw_angle(x, y)
    data = {}
    data["waypoints"] = []
    for i in range(len(x)):
        data["waypoints"].append({
            "number": int(i),
            "x": float(x[i]),
            "y": float(y[i]),
            "z": float(depth),
            "yaw": float(yaw[i]),
        })
        print('Generated waypoint nr. ', i)

    return data


def test_and_plot(factor=1.0, x_offset=0.7, y_offset=2.0, number=100):
    t = np.linspace(0, 2*np.pi, num=100, endpoint=False)

    # 8 shape using Bernoulli
    x, y = lemniscate_of_bernoulli(t)
    x, y = scale_and_move(x, y, x_offset, y_offset, factor)
    yaw = get_yaw_angle(x, y)

    # 8 shape using Gerono
    x_gerono, y_gerono = lemniscate_of_gerono(t)
    x_gerono, y_gerono = scale_and_move(x_gerono, y_gerono, x_offset, y_offset,
                                        factor)
    # reverse to get same direction as Bernoulli lemniscate
    # x_gerono = x_gerono[::-1]
    # y_gerono = y_gerono[::-1]
    yaw_gerono = get_yaw_angle(x_gerono, y_gerono)

    # super ellipse
    # TODO: here, the points aren't even close to equidistant!
    n = 4.0
    # length=2*a, width=2*b
    a = 0.5
    b = 1.0
    x_ellipse, y_ellipse = super_ellipse(t, a, b, n)
    x_ellipse, y_ellipse = scale_and_move(x_ellipse,
                                          y_ellipse,
                                          x_offset,
                                          y_offset,
                                          factor=1.0)
    yaw_ellipse = get_yaw_angle(x_ellipse, y_ellipse)

    # circle
    x_circle, y_circle = circle(t)
    x_circle, y_circle = scale_and_move(x_circle,
                                        y_circle,
                                        x_offset,
                                        y_offset,
                                        factor=0.5)
    yaw_circle = get_yaw_angle(x_circle, y_circle)

    fig1 = plt.figure()
    ax = fig1.add_subplot(111)
    plt.plot(x, y, ".", label='Bernoulli')
    plt.plot(x_gerono, y_gerono, ".", label='Gerono')
    plt.plot(x_circle, y_circle, ".", label='Circle')
    plt.plot(x_ellipse, y_ellipse, ".", label='Super Ellipse')
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.legend()
    # plt.xlim([0.0, 2.0])
    # plt.ylim([0.0, 4.0])
    ax.set_aspect('equal', adjustable='box')

    fig2 = plt.figure()
    ax = fig2.add_subplot(111)
    plt.plot(t, np.rad2deg(yaw), label='Bernoulli')
    plt.plot(t, np.rad2deg(yaw_gerono), label='Gerono')
    plt.plot(t, np.rad2deg(yaw_circle), label='Circle')
    plt.plot(t, np.rad2deg(yaw_ellipse), label='Super Ellipse')
    plt.xlabel("t")
    plt.ylabel("yaw angle")
    plt.title("Yaw angle over paramterization t")
    plt.legend()
    plt.show()


def main():

    factor = 0.9
    # test factor
    # test_and_plot(factor)

    filename = "waypoints.yaml"
    with open(filename, "w") as file_handle:
        data = generate_waypoints(factor)  # always generating bernoulli curve!
        yaml.dump(data, file_handle)
        print("Created file '{}'".format(os.path.join(os.getcwd(), filename)))
        print("Probably you want to move it to this package's config directory")


if __name__ == "__main__":
    main()
