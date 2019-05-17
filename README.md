# Kinematics

## Supported Platforms

Mac OS X 10.9.5

## Instructions for Execution

``` bash
$ make
$ ./as4 <file>
```

We provided a sample input file which can be run with `./as4 input.txt`.

## Input Format

The input consists of a series of commands, each on its own separate line. A command consists of a flag followed by a series of arguments. These commands define the scene on which the program executes.

### Flags
- -arm [joint/]length ... (for joint types, use 'pm' for prismatic joints, 'pn' for pin joints, and 'ba' for ball joints)
- -path a b (where a and b are coefficients defining the surface described by equation z = ax^3 + by^3)
- -cir r (where r is the radius of the cross-section of a vertical cylinder centered at the origin)
- -ell a b (where a and b define the minor and major radii of an ellipse centered at the origin)

*Note: prismatic and pin joints are currently unimplemented.

The resulting path is the intersection of the cubic surface and the volume specified by the cir or ell flags.
