#pragma once

#include "common.h"
#include "vec.h"
#include "quat.h"
#include "array.h"


//--------------------------------------

// Compute forward kinematics for all joints
void forward_kinematics_full(
    slice1d<vec3> global_bone_positions,
    slice1d<quat> global_bone_rotations,
    const slice1d<vec3> local_bone_positions,
    const slice1d<quat> local_bone_rotations,
    const slice1d<int> bone_parents)
{
    for (int i = 0; i < bone_parents.size; i++)
    {
        // Assumes bones are always sorted from root onwards
        assert(bone_parents(i) < i);

        if (bone_parents(i) == -1)
        {
            global_bone_positions(i) = local_bone_positions(i);
            global_bone_rotations(i) = local_bone_rotations(i);
        }
        else
        {
            vec3 parent_position = global_bone_positions(bone_parents(i));
            quat parent_rotation = global_bone_rotations(bone_parents(i));
            global_bone_positions(i) = quat_mul_vec3(parent_rotation, local_bone_positions(i)) + parent_position;
            global_bone_rotations(i) = quat_mul(parent_rotation, local_bone_rotations(i));
        }
    }
}

// Here we using a simple recursive version of forward kinematics for calculation only one bone
void forward_kinematics(
    vec3& bone_position,
    quat& bone_rotation,
    const slice1d<vec3> bone_positions,
    const slice1d<quat> bone_rotations,
    const slice1d<int> bone_parents,
    const int bone)
{
    if (bone_parents(bone) != -1)
    {
        vec3 parent_position;
        quat parent_rotation;

        forward_kinematics(
            parent_position,
            parent_rotation,
            bone_positions,
            bone_rotations,
            bone_parents,
            bone_parents(bone));

        bone_position = quat_mul_vec3(parent_rotation, bone_positions(bone)) + parent_position;
        bone_rotation = quat_mul(parent_rotation, bone_rotations(bone));
    }
    else
    {
        bone_position = bone_positions(bone);
        bone_rotation = bone_rotations(bone);
    }
}

// Forward kinematics but also compute the velocities
void forward_kinematics_velocity(
    vec3& bone_position,
    vec3& bone_velocity,
    quat& bone_rotation,
    vec3& bone_angular_velocity,
    const slice1d<vec3> bone_positions,
    const slice1d<vec3> bone_velocities,
    const slice1d<quat> bone_rotations,
    const slice1d<vec3> bone_angular_velocities,
    const slice1d<int> bone_parents,
    const int bone)
{
    if (bone_parents(bone) != -1)
    {
        vec3 parent_position;
        vec3 parent_velocity;
        quat parent_rotation;
        vec3 parent_angular_velocity;

        forward_kinematics_velocity(
            parent_position,
            parent_velocity,
            parent_rotation,
            parent_angular_velocity,
            bone_positions,
            bone_velocities,
            bone_rotations,
            bone_angular_velocities,
            bone_parents,
            bone_parents(bone));

        bone_position = quat_mul_vec3(parent_rotation, bone_positions(bone)) + parent_position;
        bone_velocity =
            parent_velocity +
            quat_mul_vec3(parent_rotation, bone_velocities(bone)) +
            cross(parent_angular_velocity, quat_mul_vec3(parent_rotation, bone_positions(bone)));
        bone_rotation = quat_mul(parent_rotation, bone_rotations(bone));
        bone_angular_velocity = quat_mul_vec3(parent_rotation, bone_angular_velocities(bone)) + parent_angular_velocity;
    }
    else
    {
        bone_position = bone_positions(bone);
        bone_velocity = bone_velocities(bone);
        bone_rotation = bone_rotations(bone);
        bone_angular_velocity = bone_angular_velocities(bone);
    }
}
