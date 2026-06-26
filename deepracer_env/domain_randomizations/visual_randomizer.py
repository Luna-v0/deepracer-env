#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#                                                                               #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#   You may not use this file except in compliance with the License.            #
#   You may obtain a copy of the License at                                     #
#                                                                               #
#       http://www.apache.org/licenses/LICENSE-2.0                              #
#                                                                               #
#   Unless required by applicable law or agreed to in writing, software         #
#   distributed under the License is distributed on an "AS IS" BASIS,           #
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    #
#   See the License for the specific language governing permissions and         #
#   limitations under the License.                                              #
#################################################################################
"""Per-episode visual domain randomization for the DeepRacer track.

This recolors the track model's visuals every episode reset (gated behind the
``GYM_DR_VISUAL_DR`` env var) so a camera-based RL policy generalizes across
visual appearances (sim2real).

WHAT CAN AND CANNOT BE REACHED (Gazebo Classic + deepracer system plugin)
------------------------------------------------------------------------
The track world (e.g. ``simulation/worlds/2022_april_open.world``) contains only
three things visually: the ``sun``/point ``light``, a ``<scene>`` block with a
``<sky/>`` element + ambient, and the ``racetrack`` model. The ``racetrack``
model (see ``simulation/models/2022_april_open/2022_april_open_model.sdf``) has
exactly two links, each with a single visual:

* ``track::visual``      -- the full track-surface mesh. The lane/edge/center
                            lines (``ol``/``il``/``cl`` sub-materials) and the
                            road surface (``road``/``field``) are baked into this
                            ONE mesh/visual, so the visual-color service can only
                            set ONE override color for the whole surface; it
                            cannot recolor the lines independently of the road.
* ``background::visual``  -- the surrounding backdrop mesh. In these worlds this
                            mesh IS the effective "ground plane + walls + sky"
                            surround the camera sees beyond the track, so we
                            treat it as the ground/wall/background channel.

Therefore:
* RECOLORABLE via ``set_visual_colors``: the track surface (``track::visual``)
  and the background/ground/wall surround (``background::visual``).
* NOT REACHABLE via the visual-color service: Gazebo's ``<sky/>`` skybox (no
  visual; rendered by the sky plugin), the ``<scene>`` ``<ambient>`` value, and
  the ``sun``/point lights (those are lights, not visuals -- the plugin exposes
  no SetLight color service). We approximate "background/sky" by recoloring
  ``background::visual`` (full hue variety), which is what dominates the camera
  frame above/around the track. The lane-vs-road contrast is preserved by NOT
  splitting them (they're one visual) and instead keeping the track surface and
  background visually distinct (different randomized colors per episode).

There are no separate ``ground_plane`` or ``wall`` models to recolor in these
worlds; if a world is later authored with such models, ``randomize`` simply
recolors whatever links the track model exposes and logs/skips anything missing.
"""

import logging

from std_msgs.msg import ColorRGBA

from deepracer_env.gazebo_utils.model_updater import ModelUpdater
from deepracer_env.gazebo_tracker.trackers.set_visual_color_tracker import SetVisualColorTracker
from deepracer_env.track_geom.constants import RACETRACK_MODEL_NAME
from deepracer_env.log_handler.logger import Logger

LOG = Logger(__name__, logging.INFO).get_logger()


class VisualRandomizer(object):
    """Recolor the track model's visuals once per episode reset.

    On construction it discovers the track model's visuals (reusing
    :meth:`ModelUpdater.get_model_visuals`, which does the
    GetModelProperties -> GetVisualNames plumbing and returns the exact
    qualified link/visual name strings the Gazebo plugin expects). It is
    defensive: any discovery failure leaves the randomizer inert (``randomize``
    becomes a no-op) so existing behavior is never broken.

    Recoloring is applied through :class:`SetVisualColorTracker` (blocking), the
    same path :class:`SetVisualColorTracker` exposes for visual recolors, so the
    SetVisualColors request is constructed exactly as the plugin expects.
    """

    # Link-name substrings used to classify discovered visuals into channels.
    # The match is on the (possibly model-qualified) link name, e.g.
    # "racetrack::track" / "racetrack::background".
    _TRACK_LINK_KEYWORDS = ("track",)
    _BACKGROUND_LINK_KEYWORDS = ("background", "ground", "wall", "sky")

    def __init__(self, model_name=RACETRACK_MODEL_NAME):
        """
        Args:
            model_name (str): Gazebo model name of the track (default 'racetrack',
                              the name the world include assigns; see world_swap).
        """
        self._model_name = model_name
        # List of (link_name, visual_name) tuples per channel, populated from the
        # exact names the GetVisualNames service returned (never guessed).
        self._track_visuals = []
        self._background_visuals = []
        self._enabled = False

        try:
            self._set_visual_color_tracker = SetVisualColorTracker.get_instance()
        except Exception as ex:  # noqa: BLE001 - stay inert on any setup failure
            LOG.warning("VisualRandomizer: could not get SetVisualColorTracker, "
                        "visual DR disabled: %s", ex)
            return

        self._discover_visuals()

    def _discover_visuals(self):
        """Discover and classify the track model's visuals.

        Reuses ModelUpdater.get_model_visuals which returns a GetVisuals response
        carrying parallel ``link_names`` / ``visual_names`` lists (the qualified
        names the plugin resolved). We classify each (link, visual) pair by link
        name into the track-surface vs background channels. Defensive: on any
        failure we log and leave the randomizer inert.
        """
        try:
            visuals = ModelUpdater.get_instance().get_model_visuals(self._model_name)
        except Exception as ex:  # noqa: BLE001
            LOG.warning("VisualRandomizer: failed to discover visuals for model "
                        "'%s', visual DR disabled: %s", self._model_name, ex)
            return

        link_names = list(getattr(visuals, "link_names", []) or [])
        visual_names = list(getattr(visuals, "visual_names", []) or [])
        if not link_names or len(link_names) != len(visual_names):
            LOG.warning("VisualRandomizer: no/uneven visuals discovered for model "
                        "'%s' (links=%d visuals=%d), visual DR disabled.",
                        self._model_name, len(link_names), len(visual_names))
            return

        for link_name, visual_name in zip(link_names, visual_names):
            lname = link_name.lower()
            if any(kw in lname for kw in self._BACKGROUND_LINK_KEYWORDS):
                self._background_visuals.append((link_name, visual_name))
            elif any(kw in lname for kw in self._TRACK_LINK_KEYWORDS):
                self._track_visuals.append((link_name, visual_name))
            else:
                # Unknown link: default it to the track-surface channel so it
                # still gets randomized rather than silently ignored.
                LOG.info("VisualRandomizer: unclassified link '%s' (visual '%s'); "
                         "treating as track surface.", link_name, visual_name)
                self._track_visuals.append((link_name, visual_name))

        self._enabled = bool(self._track_visuals or self._background_visuals)
        if not self._enabled:
            LOG.warning("VisualRandomizer: discovered no recolorable visuals for "
                        "model '%s', visual DR disabled.", self._model_name)
        else:
            LOG.info("VisualRandomizer: enabled for model '%s' "
                     "(track visuals=%d, background visuals=%d).",
                     self._model_name, len(self._track_visuals),
                     len(self._background_visuals))

    @staticmethod
    def _rgba(rng, lo=0.0, hi=1.0, alpha=1.0):
        """Sample a ColorRGBA with each channel uniform in [lo, hi]."""
        return ColorRGBA(r=float(rng.uniform(lo, hi)),
                         g=float(rng.uniform(lo, hi)),
                         b=float(rng.uniform(lo, hi)),
                         a=float(alpha))

    @staticmethod
    def _color_distance(c1, c2):
        """Squared RGB distance between two ColorRGBA (contrast proxy)."""
        return (c1.r - c2.r) ** 2 + (c1.g - c2.g) ** 2 + (c1.b - c2.b) ** 2

    def _apply(self, visuals, ambient, diffuse, specular=None, emissive=None):
        """Recolor a set of (link, visual) pairs (blocking).

        Mirrors SetVisualColorTracker.set_visual_color's per-visual call
        convention: visual_name, link_name, ambient, diffuse, specular,
        emissive, blocking. We call it once per visual with blocking=True so the
        recolor is flushed to Gazebo at reset time (not deferred to the next
        tracker update tick).
        """
        if specular is None:
            specular = ColorRGBA(r=0.1, g=0.1, b=0.1, a=1.0)
        if emissive is None:
            emissive = ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)
        for link_name, visual_name in visuals:
            try:
                res = self._set_visual_color_tracker.set_visual_color(
                    visual_name=visual_name,
                    link_name=link_name,
                    ambient=ambient,
                    diffuse=diffuse,
                    specular=specular,
                    emissive=emissive,
                    blocking=True)
                if res is not None and not getattr(res, "success", True):
                    LOG.warning("VisualRandomizer: set_visual_color failed for "
                                "'%s' (link '%s'): %s", visual_name, link_name,
                                getattr(res, "status_message", ""))
            except Exception as ex:  # noqa: BLE001 - skip a bad visual, keep going
                LOG.warning("VisualRandomizer: exception recoloring '%s' "
                            "(link '%s'), skipping: %s", visual_name, link_name, ex)

    def randomize(self, rng):
        """Sample new colors and recolor the discovered visuals.

        Args:
            rng (numpy.random.Generator): RNG to sample colors from (caller owns
                seeding so the schedule is reproducible).

        No-op if discovery found nothing recolorable (keeps existing behavior).
        """
        if not self._enabled:
            return

        # Track surface: full hue variety. (Lines share this visual, so we can't
        # split them; we keep the surface distinct from the background instead.)
        track_diffuse = self._rgba(rng)
        # Track ambient is a dimmer version of diffuse (typical material look).
        track_ambient = ColorRGBA(r=track_diffuse.r * 0.6, g=track_diffuse.g * 0.6,
                                  b=track_diffuse.b * 0.6, a=1.0)

        # Background / ground / wall / sky surround: full hue variety, but
        # resampled until it is reasonably distinct from the track surface so the
        # track stays visually separable in the camera frame.
        bg_diffuse = self._rgba(rng)
        for _ in range(8):
            if self._color_distance(bg_diffuse, track_diffuse) >= 0.10:
                break
            bg_diffuse = self._rgba(rng)
        bg_ambient = ColorRGBA(r=bg_diffuse.r * 0.6, g=bg_diffuse.g * 0.6,
                               b=bg_diffuse.b * 0.6, a=1.0)

        self._apply(self._track_visuals, ambient=track_ambient, diffuse=track_diffuse)
        self._apply(self._background_visuals, ambient=bg_ambient, diffuse=bg_diffuse)
