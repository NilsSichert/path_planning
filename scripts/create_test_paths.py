#!/usr/bin/env python
import yaml
import numpy as np
import os
import path_planning.path_generation as pg
import matplotlib.pyplot as plt


def generate_waypoints(factor=1.0,
                       x_offset=0.7,
                       y_offset=2.0,
                       depth=-0.5,
                       number=100):
    # don't want duplicate of first point as last point -> endpoint=False
    t = np.linspace(0, 2.0 * np.pi, num=number, endpoint=False)
    x_raw, y_raw = pg.lemniscate_of_bernoulli(t)
    x, y = pg.scale_and_move(x_raw,
                             y_raw,
                             x_offset=x_offset,
                             y_offset=y_offset,
                             factor=1.0)

    yaw = pg.get_yaw_angle(x, y)
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
    t = np.linspace(0, 2 * np.pi, num=100, endpoint=False)

    # 8 shape using Bernoulli
    x, y = pg.lemniscate_of_bernoulli(t)
    x, y = pg.scale_and_move(x, y, x_offset, y_offset, factor)
    yaw = pg.get_yaw_angle(x, y)

    # 8 shape using Gerono
    x_gerono, y_gerono = pg.lemniscate_of_gerono(t)
    x_gerono, y_gerono = pg.scale_and_move(x_gerono, y_gerono, x_offset,
                                           y_offset, factor)
    # reverse to get same direction as Bernoulli lemniscate
    # x_gerono = x_gerono[::-1]
    # y_gerono = y_gerono[::-1]
    yaw_gerono = pg.get_yaw_angle(x_gerono, y_gerono)

    # super ellipse
    # TODO: here, the points aren't even close to equidistant!
    n = 4.0
    # length=2*a, width=2*b
    a = 0.5
    b = 1.0
    x_ellipse, y_ellipse = pg.super_ellipse(t, a, b, n)
    x_ellipse, y_ellipse = pg.scale_and_move(x_ellipse,
                                             y_ellipse,
                                             x_offset,
                                             y_offset,
                                             factor=1.0)
    yaw_ellipse = pg.get_yaw_angle(x_ellipse, y_ellipse)

    # circle
    x_circle, y_circle = pg.circle(t)
    x_circle, y_circle = pg.scale_and_move(x_circle,
                                           y_circle,
                                           x_offset,
                                           y_offset,
                                           factor=0.5)
    yaw_circle = pg.get_yaw_angle(x_circle, y_circle)

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
