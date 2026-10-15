# ADR-0012: ROS Domain Chosen by the Agent

**Status:** Accepted
**Date:** 2026-10-14
**Related:** ADR-0002 (micro-ROS transport); host repository bug M3 (the
robot is on DDS domain 0)

## Context

The firmware's micro-ROS client never set a ROS domain, so its session
always asked the agent for domain 0, the micro-ROS default. The host stack
takes its domain from `ROS_DOMAIN_ID` (one value per robot, in its
`deploy/.env`). When the host moved to domain 11, the agent created the
firmware's participant on 0: `/odom` disappeared, `/cmd_vel` had no
subscriber, and the robot could not drive.

The ESP32 has no environment to read. A domain pinned at build time needs a
rebuild and a reflash for every change, and it can disagree with the host's
setting. A domain stored on the chip (NVS, set over the tuning CLI) is a
second copy of the same setting, with its own tool.

Micro XRCE-DDS reserves domain ID 255: when a client creates its
participant on 255, the agent (v2.4.1 and newer) uses its
`XRCE_DOMAIN_ID_OVERRIDE` environment variable instead. The micro-ROS
agent's graph manager follows the participant's actual domain.

## Decision

- The firmware requests `CONFIG_UROS_DOMAIN_ID` through
  `rcl_init_options_set_domain_id` when each session starts. The default
  is 255: the agent chooses the domain.
- The host starts the agent with `XRCE_DOMAIN_ID_OVERRIDE` set to its own
  `ROS_DOMAIN_ID`, so the chassis is always on the stack's domain, and the
  robot's `deploy/.env` is the only place the domain is set.
- 0-232 remain selectable in menuconfig, to pin the domain for a bench
  agent without the override. Values 233-254 fail the build.

## Consequences

- Changing the robot's domain is a host setting and a restart; no reflash.
- The agent logs `Overriding Micro XRCE-DDS Client DOMAIN_ID` with the
  domain it chose, each time a session starts.
- An agent older than v2.4.1 cannot create domain 255: the session never
  starts and the robot does not drive (the safe direction). The host image
  builds micro-ROS-Agent `jazzy`, which bundles v2.4.3.
- An agent started without the variable uses domain 0.
