#!/usr/bin/env python
import os
import xml.etree.cElementTree as ET

import numpy as np


def largest_prime_factor(n):
    i = 2
    while i * i <= n:
        if n % i:
            i += 1
        else:
            n //= i
    return n


def camera_instrinsics_to_xml(camera_matrix, distortion_parameters, pixel_size_mm, output_dir):
    root = ET.Element("Camera")

    ET.SubElement(root, "Cx").text = "{0}".format(camera_matrix[0][2])
    ET.SubElement(root, "Cy").text = "{0}".format(camera_matrix[1][2])

    ET.SubElement(root, "Fx").text = "{0}".format(camera_matrix[0][0])
    ET.SubElement(root, "Fy").text = "{0}".format(camera_matrix[1][1])

    ET.SubElement(root, "K1").text = "{0}".format(distortion_parameters[0])
    ET.SubElement(root, "K2").text = "{0}".format(distortion_parameters[1])
    ET.SubElement(root, "K3").text = "{0}".format(distortion_parameters[4])

    ET.SubElement(root, "P1").text = "{0}".format(distortion_parameters[2])
    ET.SubElement(root, "P2").text = "{0}".format(distortion_parameters[3])

    ET.SubElement(root, "Sx").text = "{0}".format(pixel_size_mm)
    ET.SubElement(root, "Sy").text = "{0}".format(pixel_size_mm)

    tree = ET.ElementTree(root)

    os.makedirs(output_dir, exist_ok=True)
    tree.write(os.path.join(output_dir, "camera_intrinsics.xml"))


def camera_intrinsic_from_xml(path):
    root = ET.parse(path).getroot()

    cx = float(root[0].text)
    cy = float(root[1].text)

    fx = float(root[2].text)
    fy = float(root[3].text)

    k1 = float(root[4].text)
    k2 = float(root[5].text)
    k3 = float(root[6].text)

    p1 = float(root[7].text)
    p2 = float(root[8].text)

    sx = float(root[9].text)
    sy = float(root[10].text)

    camera_matrix = np.array([fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]).reshape(3, 3)
    distortion_parameters = np.array([k1, k2, p1, p2, k3])
    pixel_size_mm = sx

    return camera_matrix, distortion_parameters, pixel_size_mm


def stereo_camera_extrinsics_to_xml(rotation_matrix, translation_vector, output_dir):
    root = ET.Element("StereoCamera")

    ET.SubElement(root, "R11").text = "{0}".format(rotation_matrix[0][0])
    ET.SubElement(root, "R12").text = "{0}".format(rotation_matrix[0][1])
    ET.SubElement(root, "R13").text = "{0}".format(rotation_matrix[0][2])

    ET.SubElement(root, "R21").text = "{0}".format(rotation_matrix[1][0])
    ET.SubElement(root, "R22").text = "{0}".format(rotation_matrix[1][1])
    ET.SubElement(root, "R23").text = "{0}".format(rotation_matrix[1][2])

    ET.SubElement(root, "R31").text = "{0}".format(rotation_matrix[2][0])
    ET.SubElement(root, "R32").text = "{0}".format(rotation_matrix[2][1])
    ET.SubElement(root, "R33").text = "{0}".format(rotation_matrix[2][2])

    ET.SubElement(root, "Tx").text = "{0}".format(translation_vector[0])
    ET.SubElement(root, "Ty").text = "{0}".format(translation_vector[1])
    ET.SubElement(root, "Tz").text = "{0}".format(translation_vector[2])

    tree = ET.ElementTree(root)

    os.makedirs(output_dir, exist_ok=True)
    tree.write(os.path.join(output_dir, "stereo_camera_extrinsics.xml"))


def stereo_camera_extrinsics_from_xml(path):
    root = ET.parse(path).getroot()

    r11 = float(root[0].text)
    r12 = float(root[1].text)
    r13 = float(root[2].text)

    r21 = float(root[3].text)
    r22 = float(root[4].text)
    r23 = float(root[5].text)

    r31 = float(root[6].text)
    r32 = float(root[7].text)
    r33 = float(root[8].text)

    tx = float(root[9].text)
    ty = float(root[10].text)
    tz = float(root[11].text)

    rotation_matrix = np.array([r11, r12, r13, r21, r22, r23, r31, r32, r33]).reshape(3, 3)
    translation_vector = np.array([tx, ty, tz])

    return rotation_matrix, translation_vector