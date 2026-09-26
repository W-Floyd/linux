.. SPDX-License-Identifier: GPL-2.0 OR GFDL-1.1-no-invariants-or-later

.. _media_metadata_layouts:

Metadata Layouts
----------------

The :ref:`metadata layout control <image_source_control_metadata_layout>`
specifies the exact layout of the metadata stream while the a :ref:`generic
metadata mbus code <media-bus-format-generic-meta>` on the subdevice pads
only describe the size of the :term:`Data Unit`.

.. _media-metadata-layout-ccs:

MIPI CCS Embedded Data Layout (``V4L2_METADATA_LAYOUT_CCS``)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`MIPI CCS <https://www.mipi.org/specifications/camera-command-set>`_ defines a
metadata layout for sensor embedded data, identified by
``V4L2_CID_METADATA_LAYOUT`` control value ``V4L2_METADATA_LAYOUT_CCS``, which
is used to store the register configuration used for capturing a given
frame. The layout itself is defined in the CCS specification.

The CCS embedded data format (code ``0xa``) definition includes three levels:

1. Padding within CSI-2 bus :term:`Data Unit` as documented in the MIPI CCS
   specification.

2. The tagged data format as documented in the MIPI CCS specification.

3. Register addresses and register documentation as documented in the MIPI CCS
   specification.

The ``V4L2_METADATA_LAYOUT_CCS`` metadata layout value shall be used only by
devices that fulfill all three levels above.

This metadata layout code is only used for "2-byte simplified tagged data
format" (code ``0xa``) but their use may be extended further in the future, to
cover other CCS embedded data format codes.

Also see :ref:`CCS driver documentation <media-ccs-routes>`.

.. _media-metadata-layout-ov2740:

Omnivision OV2740 Embedded Data Layout (``V4L2_METADATA_LAYOUT_OV2740``)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The Omnivision OV2740 camera sensor produces the following embedded data layout,
indicated by ``V4L2_METADATA_LAYOUT_OV2740`` metadata layout. The format
conforms to :ref:`CCS embedded data layout <media-metadata-layout-ccs>` up to
level 1.

.. flat-table:: Omnivision OV2740 Embedded Data Layout. Octets at indices marked
                reserved or unused have been omitted from the table. The values
                are in big endian byte order.
    :header-rows: 1

    * - Offset
      - Size in bits (active bits if not the same as size)
      - Content description
    * - 4
      - 16 (10--0)
      - Analogue gain
    * - 6
      - 16
      - Coarse integration time
    * - 10
      - 8
      - DPC correction threshold bits 9--2
    * - 15
      - 16
      - Output image width
    * - 17
      - 16
      - Output image height
    * - 23
      - 8
      - MIPI header revision number (2)
    * - 31
      - 8
      - Vertical (bit 1) and horizontal flip (bit 0)
    * - 32
      - 8
      - Frame duration A
    * - 33
      - 8
      - Frame duration B
    * - 34
      - 8
      - Context count (2)
    * - 35
      - 8
      - Context select
    * - 54
      - 8
      - Data pedestal bits 9--2
    * - 63
      - 8
      - Frame average bits 9--2
    * - 64
      - 16
      - Digital gain red
    * - 66
      - 16
      - Digital gain greenr
    * - 68
      - 16
      - Digital gain blue
    * - 70
      - 16
      - Digital gain greenb
    * - 89
      - 8
      - Frame counter (starts at 1, wraps to 0 after 255)

.. _media-metadata-layout-s5kjn1-pdaf:

Samsung S5KJN1 Phase Detection Data Layout (``V4L2_METADATA_LAYOUT_S5KJN1_PDAF``)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The Samsung S5KJN1 (and the S5KJNS, which shares its register map) has phase
detection (PD) pixels: pairs of pixels shielded to see opposite halves of the
lens pupil, called left (L) and right (R) pixels here. In its 4080x3072 mode it
sends the values of those pixels, as it reads them out, in a stream of their
own next to the image, indicated by the ``V4L2_METADATA_LAYOUT_S5KJN1_PDAF``
metadata layout. The phase difference between the L and R signals of a region
is proportional to its defocus; computing it is left to the user space.

The PD pixels sit on a grid of 8x8 pixel blocks, the first block starting at
column 8 and row 8 of the 4080x3072 output image, 508 blocks across and 382
down. Each block has four L/R pairs. Relative to the block's top left pixel:

.. flat-table:: S5KJN1 PD pixel positions within a block (column, row)
    :header-rows: 1

    * - Pair
      - R pixel
      - L pixel
    * - 0
      - (2, 0)
      - (3, 0)
    * - 1
      - (0, 3)
      - (1, 3)
    * - 2
      - (4, 4)
      - (5, 4)
    * - 3
      - (6, 7)
      - (7, 7)

The PD stream is 508 samples wide and 3056 (8 x 382) lines high, with 10 bits
per sample, packed as :ref:`V4L2_META_FMT_GENERIC_CSI2_10
<v4l2-meta-fmt-generic-csi2-10>` (MIPI CSI-2 RAW10 packing). Sample ``x`` of a
line is the PD pixel of block column ``x``. Lines come in groups of eight, one
group per block row, in this order:

.. flat-table:: S5KJN1 PD stream lines within a group of eight
    :header-rows: 1

    * - Line
      - Content
    * - 0
      - R pixel of pair 0
    * - 1
      - L pixel of pair 0
    * - 2
      - R pixel of pair 1
    * - 3
      - L pixel of pair 1
    * - 4
      - R pixel of pair 2
    * - 5
      - L pixel of pair 2
    * - 6
      - R pixel of pair 3
    * - 7
      - L pixel of pair 3

The pixel positions are given for the image with neither flip applied. The
values include the sensor's black level.
