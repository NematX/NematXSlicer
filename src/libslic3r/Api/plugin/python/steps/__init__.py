#/|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
#/|/
#/|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#/|/

"""Public imports for the Python slicing-step contexts and service adapters.

Import a context from this package when implementing the corresponding step.
Each context module documents the data borrowed for that call and the mutations
its callbacks permit. Service modules additionally provide session providers and
consumer views; importing them does not itself register or execute a plugin.
"""

from .layer_height import LayerConfigRange, LayerDescriptor, LayerHeightContext
from .slicing import SlicingContext, SlicingLayerRange, SlicingVolumeRegion
from .post_slicing import PostSlicingContext
from .perimeter import PerimeterContext, PerimeterGenerationContextView, PerimeterNodeView
from .perimeter_module import PerimeterModuleContext, PublishedPerimeterGenerationModule
from .seam_placer import (
    SeamPlacer,
    SeamPlacerContext,
    SeamPlacerSession,
    make_seam_placer_instance,
)
from .gcode_firmware import (
    GCodeFirmwareContext,
    GCodeFirmwareSession,
    GCodeFirmwareView,
    GCodeScriptConfig,
    GCodeScriptContext,
    GCodeScriptProcessorView,
    make_gcode_firmware_instance,
)
from slic3r_gcode_reader_service import GCodeReaderService
from .gcode_thumbnail import (
    GCodeThumbnailContext,
    GCodeThumbnailEncodedImage,
    GCodeThumbnailEncoderSession,
    GCodeThumbnailEncoderView,
    GCodeThumbnailImage,
    GCodeThumbnailEncoder,
    GCodeThumbnailRendererView,
    GCodeThumbnailRequest,
    make_gcode_thumbnail_encoder_instance,
)
from .post_perimeter import PostPerimeterContext
from .post_infill import PostInfillContext
from .surface_generation import SurfaceGenerationContext
from .execution import (
    ExtrusionEditContext,
    ExtrusionSimplificationContext,
    GCodeContext,
    InfillContext,
    InfillGroupingContext,
    InfillPatternContext,
    InfillSurfaceRecipeModifierContext,
    LayerExtrusionEditContext,
    LayerStitchingContext,
    OrderingContext,
    FinalizeSlicingContext,
    PreInfillContext,
    PrePerimeterContext,
    SkirtBrimContext,
    SupportContext,
    SupportDemandContext,
    SupportSpotContext,
    SupportToolpathContext,
    WipeTowerContext,
)

__all__ = [
    "LayerConfigRange",
    "LayerDescriptor",
    "LayerHeightContext",
    "SlicingContext",
    "SlicingLayerRange",
    "SlicingVolumeRegion",
    "PostSlicingContext",
    "PerimeterContext",
    "PerimeterGenerationContextView",
    "PerimeterNodeView",
    "PerimeterModuleContext",
    "PublishedPerimeterGenerationModule",
    "SeamPlacer",
    "SeamPlacerContext",
    "SeamPlacerSession",
    "make_seam_placer_instance",
    "GCodeFirmwareContext",
    "GCodeFirmwareSession",
    "GCodeFirmwareView",
    "GCodeScriptConfig",
    "GCodeScriptContext",
    "GCodeScriptProcessorView",
    "GCodeReaderService",
    "make_gcode_firmware_instance",
    "GCodeThumbnailContext",
    "GCodeThumbnailEncodedImage",
    "GCodeThumbnailEncoderSession",
    "GCodeThumbnailEncoderView",
    "GCodeThumbnailImage",
    "GCodeThumbnailEncoder",
    "GCodeThumbnailRendererView",
    "GCodeThumbnailRequest",
    "make_gcode_thumbnail_encoder_instance",
    "PostPerimeterContext",
    "PostInfillContext",
    "SurfaceGenerationContext",
    "ExtrusionEditContext",
    "ExtrusionSimplificationContext",
    "GCodeContext",
    "InfillContext",
    "InfillGroupingContext",
    "InfillPatternContext",
    "InfillSurfaceRecipeModifierContext",
    "LayerExtrusionEditContext",
    "LayerStitchingContext",
    "OrderingContext",
    "FinalizeSlicingContext",
    "PreInfillContext",
    "PrePerimeterContext",
    "SkirtBrimContext",
    "SupportContext",
    "SupportDemandContext",
    "SupportSpotContext",
    "SupportToolpathContext",
    "WipeTowerContext",
]
