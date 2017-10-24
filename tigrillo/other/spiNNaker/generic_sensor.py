"""
Generic analog sensor on the Tigrillo robot
"""

from spynnaker8.utilities import DataHolder
from pacman.model.graphs.application import ApplicationSpiNNakerLinkVertex
from pacman.model.routing_info import BaseKeyAndMask
from pacman.model.constraints.key_allocator_constraints import FixedKeyAndMaskConstraint
from spinn_front_end_common.abstract_models import AbstractProvidesOutgoingPartitionConstraints


class AnalogDevice(ApplicationSpiNNakerLinkVertex, AbstractProvidesOutgoingPartitionConstraints):

    def __init__(self, n_neurons, spinnaker_link_id, board_address=None, label=None):

        ApplicationSpiNNakerLinkVertex.__init__(self, n_atoms=n_neurons, spinnaker_link_id=spinnaker_link_id,
                                                board_address=board_address, label=label)

    def get_outgoing_partition_constraints(self, partition):

        return [FixedKeyAndMaskConstraint([BaseKeyAndMask(0x67890000, 0xFFFF0000)])]


class AnalogDeviceDataHolder(DataHolder):

    def __init__(self, spinnaker_link_id, board_address=None, label=None):

        DataHolder.__init__(self, {"spinnaker_link_id": spinnaker_link_id,
                                   "board_address": board_address, "label": label})

    @staticmethod
    def build_model():
        return AnalogDevice
