import logging
import os
import csv
from typing import Annotated

import vtk
import qt
import ctk
import numpy as np

import slicer
from slicer.i18n import tr as _
from slicer.i18n import translate
from slicer.ScriptedLoadableModule import *
from slicer.util import VTKObservationMixin
from slicer.parameterNodeWrapper import (
    parameterNodeWrapper,
    WithinRange,
)
from slicer import vtkMRMLScalarVolumeNode


class Beta_angle_ana(ScriptedLoadableModule):
    """
    Beta_angle_ana
    A scripted loadable module template adapted and refactored from your original script.

    This class registers the module in Slicer (title, categories, help text...) and
    connects the sample-data registration to application startup.
    """

    def __init__(self, parent):
        ScriptedLoadableModule.__init__(self, parent)
        self.parent.title = _("Beta_angle_ana")
        # Keep categories concise and correct
        self.parent.categories = [ "Custome_model"]
        self.parent.dependencies = []
        self.parent.contributors = ["Elise Joulain (UNIGE)"]
        self.parent.helpText = _("""
This module assists with placing fiducials on shoulder X-rays and computing various
metrics (beta angle and related). It was refactored for clarity and stability.
""")
        self.parent.acknowledgementText = _(
            """
Developed at UNIGE. Based on ScriptedLoadableModule examples from 3D Slicer.
"""
        )

        # Register sample data after startup (optional)
        slicer.app.connect("startupCompleted()", registerSampleData)


def registerSampleData():
    """Register optional sample data shown in the Sample Data module."""
    try:
        import SampleData
    except Exception:
        return

    iconsPath = os.path.join(os.path.dirname(__file__), "Resources", "Icons")

    SampleData.SampleDataLogic.registerCustomSampleDataSource(
        category="Beta_angle_ana",
        sampleName="Beta_angle_ana1",
        thumbnailFileName=os.path.join(iconsPath, "Beta_angle_ana1.png"),
        uris="https://github.com/Slicer/SlicerTestingData/releases/download/SHA256/998cb522173839c78657f4bc0ea907cea09fd04e44601f17c82ea27927937b95",
        fileNames="Beta_angle_ana1.nrrd",
        checksums="SHA256:998cb522173839c78657f4bc0ea907cea09fd04e44601f17c82ea27927937b95",
        nodeNames="Beta_angle_ana1",
    )

    SampleData.SampleDataLogic.registerCustomSampleDataSource(
        category="Beta_angle_ana",
        sampleName="Beta_angle_ana2",
        thumbnailFileName=os.path.join(iconsPath, "Beta_angle_ana2.png"),
        uris="https://github.com/Slicer/SlicerTestingData/releases/download/SHA256/1a64f3f422eb3d1c9b093d1a18da354b13bcf307907c66317e2463ee530b7a97",
        fileNames="Beta_angle_ana2.nrrd",
        checksums="SHA256:1a64f3f422eb3d1c9b093d1a18da354b13bcf307907c66317e2463ee530b7a97",
        nodeNames="Beta_angle_ana2",
    )


@parameterNodeWrapper
class Beta_angle_anaParameterNode:
    """Parameter node wrapper that defines typed parameters and defaults."""

    inputVolume: vtkMRMLScalarVolumeNode
    imageThreshold: Annotated[float, WithinRange(-100, 500)] = 100
    invertThreshold: bool = False
    thresholdedVolume: vtkMRMLScalarVolumeNode
    invertedVolume: vtkMRMLScalarVolumeNode


class Beta_angle_anaWidget(ScriptedLoadableModuleWidget, VTKObservationMixin):
    """Refactored widget class: builds UI, connects callbacks and delegates heavy work to Logic."""

    def __init__(self, parent=None) -> None:
        ScriptedLoadableModuleWidget.__init__(self, parent)
        VTKObservationMixin.__init__(self)
        self.logic = None
        self.inputVolumeSelector = None
        self.markupNode = None

    def setup(self) -> None:
        ScriptedLoadableModuleWidget.setup(self)

        # Simple mapping of fiducial labels used in the UI table
        # NOTE: numbering updated (15 points total). Mapping vs the previous
        # 16-point scheme (for traceability):
        #   old P13 -> new P1   (Diaphysis proximal medial)
        #   old P14 -> new P2   (Diaphysis proximal lateral)
        #   old P15 -> new P3   (Diaphysis distal medial)
        #   old P16 -> new P4   (Diaphysis distal lateral)
        #   old P12 -> new P5   (GT lateral angle)
        #   old P9 + old P8 -> new P6  (Head-GT junction, merged duplicate point)
        #   old P10 -> new P7  (Head-calcar junction)
        #   old P11 -> new P8  (Head surface)
        #   old P1  -> new P9  (Medial fossa)
        #   old P2  -> new P10 (Lateral fossa)
        #   old P4  -> new P11 (Upper glenoid)
        #   old P3  -> new P12 (Lower glenoid)
        #   old P6  -> new P13 (Lateral acromion)
        #   old P7  -> new P14 (Medial acromion)
        #   old P5  -> new P15 (Medial part of the scapular neck)
        self.fiducialLabels = {
            "P1": "Diaphysis proximal medial - external cortex",
            "P2": "Diaphysis proximal lateral - external cortex",
            "P3": "Diaphysis distal medial - external cortex",
            "P4": "Diaphysis distal lateral - external cortex",
            "P5": "Lateral greater tuberosity (GT)",
            "P6": "Upper head-calcar junction",
            "P7": "Lower head-calcar junction",
            "P8": "Head surface",
            "P9": "Medial fossa",
            "P10": "Lateral fossa",
            "P11": "Upper glenoid",
            "P12": "Lower glenoid",
            "P13": "Lateroinferior acromion",
            "P14": "Medio-inferior acromion",
            "P15": "Medial part of the scapular neck",
        }

        # Main layout and form
        formLayout = qt.QFormLayout()
        self.layout.addLayout(formLayout)

        # Input volume selector
        self.inputVolumeSelector = slicer.qMRMLNodeComboBox()
        self.inputVolumeSelector.nodeTypes = ["vtkMRMLScalarVolumeNode"]
        self.inputVolumeSelector.setMRMLScene(slicer.mrmlScene)
        self.inputVolumeSelector.currentNodeChanged.connect(self.onInputVolumeChanged)
        self.layout.addWidget(qt.QLabel("Choose the volume on which you want to work:"))
        self.layout.addWidget(self.inputVolumeSelector)
        # Cosmetic style (optional)
        self.inputVolumeSelector.setStyleSheet("""
                    QComboBox {
                        background-color: #587880;
                        color: #587880;
                        font-weight: bold;
                        border: 1px solid #00796B;
                        padding: 3px;
                        border-radius: 4px;
                    }
                """)

        # Side selection
        self.selectionCollapsibleButton = ctk.ctkCollapsibleButton()
        self.selectionCollapsibleButton.text = "Enter the side of the inspected shoulder"
        self.layout.addWidget(self.selectionCollapsibleButton)
        self.selectionFormLayout = qt.QFormLayout(self.selectionCollapsibleButton)
        self.sideSelector = qt.QComboBox()
        self.sideSelector.addItem("Right")
        self.sideSelector.addItem("Left")
        self.selectionFormLayout.addRow("Select Side:", self.sideSelector)

        # X-ray type selection (native / anatomic prosthesis / reverse prosthesis)
        # This determines which metrics are computed in onCalculateMetrics:
        #   - Native & Anatomic prosthesis: standard metric set (unchanged)
        #   - Reverse prosthesis: standard metric set + Glenosphere Inclination (GI) + Overhang
        self.xrayTypeSelector = qt.QComboBox()
        self.xrayTypeSelector.addItem("Native")
        self.xrayTypeSelector.addItem("Anatomic prosthesis")
        self.xrayTypeSelector.addItem("Reverse prosthesis")
        self.xrayTypeSelector.setToolTip(
            "Select the type of shoulder X-ray being analyzed. "
            "For 'Reverse prosthesis', place P6 and P7 on the glenosphere rim "
            "(instead of the native humeral head contour) to enable the additional "
            "Glenosphere Inclination and Overhang measurements."
        )
        self.selectionFormLayout.addRow("X-ray type:", self.xrayTypeSelector)

        # Patient ID input
        self.patientIdLineEdit = qt.QLineEdit()
        self.patientIdLineEdit.setPlaceholderText("Enter patient ID")
        formLayout.addRow("Patient ID:", self.patientIdLineEdit)

        # Output folder selector
        self.outputDirectoryButton = ctk.ctkPathLineEdit()
        self.outputDirectoryButton.filters = ctk.ctkPathLineEdit.Dirs
        self.outputDirectoryButton.setToolTip("Select folder where results will be saved")
        self.outputDirectoryButton.currentPath = slicer.app.temporaryPath

        formLayout.addRow("Output folder:", self.outputDirectoryButton)

        # PDF for placement of the fiducial point
        self.instructionsButton = qt.QPushButton("View placement guide (PDF)")
        self.instructionsButton.setToolTip("Open the fiducial placement instructions")
        self.instructionsButton.clicked.connect(self.onOpenInstructionsPDF)
        self.layout.addWidget(self.instructionsButton)

        # Fiducial table
        self.fiducialTableCollapsible = ctk.ctkCollapsibleButton()
        self.fiducialTableCollapsible.text = "Here a table guiding the position of the needed fiducials nodes for the analysis"
        self.layout.addWidget(self.fiducialTableCollapsible)
        self.fiducialTableLayout = qt.QVBoxLayout(self.fiducialTableCollapsible)

        self.fiducialTable = qt.QTableWidget()
        self.fiducialTable.setRowCount(len(self.fiducialLabels))
        self.fiducialTable.setColumnCount(2)
        self.fiducialTable.setHorizontalHeaderLabels(["ID", "Description"])
        self.fiducialTable.horizontalHeader().setStretchLastSection(True)
        self.fiducialTable.verticalHeader().visible = False
        self.fiducialTable.setEditTriggers(qt.QAbstractItemView.NoEditTriggers)

        for i, (fidID, label) in enumerate(self.fiducialLabels.items()):
            self.fiducialTable.setItem(i, 0, qt.QTableWidgetItem(fidID))
            self.fiducialTable.setItem(i, 1, qt.QTableWidgetItem(label))

        self.fiducialTableLayout.addWidget(self.fiducialTable)


        # Analysis section
        self.beta_angleGroup = ctk.ctkCollapsibleButton()
        self.beta_angleGroup.text = "Analysis of the image"
        self.layout.addWidget(self.beta_angleGroup)
        self.beta_angleLayout = qt.QVBoxLayout(self.beta_angleGroup)
        self.beta_angleGroup.setStyleSheet("background-color: #587880; color: white;")

        self.beta_anglePlaceButton = qt.QPushButton("Place fiducials nodes")
        self.beta_anglePlaceButton.connect('clicked(bool)', lambda: self.onPlaceFiducial("beta_angle"))
        self.beta_angleLayout.addWidget(self.beta_anglePlaceButton)

        self.beta_angleComputeButton = qt.QPushButton("Compute parameters")
        self.beta_angleComputeButton.connect('clicked(bool)', lambda: self.onCalculateMetrics("beta_angle"))
        self.beta_angleLayout.addWidget(self.beta_angleComputeButton)

        # --- Toggle fiducial labels ---
        self.toggleLabelsButton = qt.QPushButton("Hide/Show Fiducial Labels")
        self.toggleLabelsButton.setStyleSheet("background-color: green; color: white;")
        self.toggleLabelsButton.toolTip = "Hide or show all fiducial labels in the scene"
        self.toggleLabelsButton.connect('clicked(bool)', self.onToggleFiducialLabels)
        self.beta_angleLayout.addWidget(self.toggleLabelsButton)

        self.beta_angleExportButton = qt.QPushButton("Export fiducials nodes and parameters")
        self.beta_angleExportButton.connect('clicked(bool)', lambda: self.onExportFiducials("beta_angle"))
        self.beta_angleLayout.addWidget(self.beta_angleExportButton)

        # Reset button
        self.resetFiducialsButton = qt.QPushButton("Reset fiducials nodes")
        self.resetFiducialsButton.toolTip = "Remove all beta_angle placed fiducial nodes"
        self.resetFiducialsButton.connect('clicked(bool)', self.onResetFiducials_preop)
        self.beta_angleLayout.addWidget(self.resetFiducialsButton)

        # Layout margins and spacing
        self.beta_angleLayout.setContentsMargins(2, 2, 2, 2)
        self.beta_angleLayout.setSpacing(3)
        self.layout.setSpacing(3)
        self.layout.setContentsMargins(2, 2, 2, 2)

        # Initialize logic
        self.logic = Beta_angle_anaLogic()


    # ---------- Callbacks and helper methods ----------
    def onInputVolumeChanged(self, volumeNode):
        if volumeNode:
            slicer.util.setSliceViewerLayers(background=volumeNode, fit=True)

    def onLoadDICOM(self):
        slicer.util.selectModule("DICOM")
        qt.QMessageBox.information(slicer.util.mainWindow(), "DICOM Import",
                                   "The DICOM browser has been opened.\nLoad your X-ray image, then return here to place fiducials.")

    def onResetFiducials_preop(self):
        scene = slicer.mrmlScene

        if not slicer.util.confirmYesNoDisplay("Are you sure you want to remove all fiducial nodes?"):
            return

        for node in list(scene.GetNodesByClass("vtkMRMLMarkupsFiducialNode")):
            name = node.GetName() or ""
            if "beta_angle" in name.lower():
                scene.RemoveNode(node)

        # Remove lines (angles / distances)
        for node in scene.GetNodesByClass("vtkMRMLMarkupsLineNode"):
            scene.RemoveNode(node)

        # Remove arcs
        for node in scene.GetNodesByClass("vtkMRMLMarkupsCurveNode"):
            scene.RemoveNode(node)

    def onPlaceFiducial(self, timepoint: str):
        """Activate placement of fiducials and create the markup node if needed."""
        nodeName = f"Fiducials_{timepoint}"
        self.markupNode = slicer.mrmlScene.GetFirstNodeByName(nodeName)
        if not self.markupNode:
            self.markupNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode", nodeName)
            # Register observer to automatically label points
            self.markupNode.AddObserver(slicer.vtkMRMLMarkupsNode.PointAddedEvent, self.onFiducialAdded)

        if "beta_angle" in timepoint.lower():
            displayNode = self.markupNode.GetDisplayNode()
            if displayNode:
                displayNode.SetSelectedColor(0, 1, 0)  # green

        # Start place mode (1 point at a time)
        slicer.modules.markups.logic().StartPlaceMode(1)

    def onFiducialAdded(self, caller, event):
        """Automatically label new fiducials P1..PN up to the number of labels provided."""
        n = caller.GetNumberOfControlPoints()
        if n <= len(self.fiducialLabels):
            fidName = f"P{n}"
            caller.SetNthControlPointLabel(n - 1, fidName)
        else:
            # Stop placement if too many points
            interactionNode = slicer.app.applicationLogic().GetInteractionNode()
            interactionNode.SetCurrentInteractionMode(slicer.vtkMRMLInteractionNode.ViewTransform)
            slicer.util.showStatusMessage("Maximum fiducials placed — placement stopped.", 3000)

    def get_input_volume_spacing(self):
        """Return spacing (x, y) of selected input volume or None with an error message."""
        try:
            volumeNode = self.inputVolumeSelector.currentNode()
            if not volumeNode:
                slicer.util.errorDisplay("No volume selected in the input selector.")
                return None

            if not volumeNode.IsA("vtkMRMLScalarVolumeNode"):
                slicer.util.errorDisplay("Selected node is not a scalar volume.")
                return None

            spacing = volumeNode.GetSpacing()  # (x, y, z)
            if tuple(spacing) == (1.0, 1.0, 1.0):
                slicer.util.warningDisplay("Warning: spacing is default (1.0 mm). Image may lack DICOM geometry.")
            return spacing[0], spacing[1]

        except Exception as e:
            slicer.util.errorDisplay(f"Error retrieving pixel spacing: {str(e)}")
            return None

    def onExportFiducials(self, timepoint: str):

        nodeName = f"Fiducials_{timepoint}"
        node = slicer.mrmlScene.GetFirstNodeByName(nodeName)

        if not node:
            slicer.util.errorDisplay(f"No fiducials placed for {timepoint}")
            return

        # --- Check metrics exist ---
        if not hasattr(self, "currentMetrics"):
            slicer.util.errorDisplay("Please compute metrics before exporting.")
            return

        # --- Ask user for folder ---
        outputDir = qt.QFileDialog.getExistingDirectory(
            slicer.util.mainWindow(),
            "Select output folder"
        )

        if not outputDir:
            slicer.util.showStatusMessage("Export cancelled.", 2000)
            return

        # --- Patient ID ---
        patientID = self.patientIdLineEdit.text.strip()
        if not patientID:
            slicer.util.errorDisplay("Please enter a patient ID before exporting.")
            return

        try:
            # --- Save fiducials + metrics together in a single CSV file ---
            self.logic.saveCombinedCSV(node, self.currentMetrics, patientID, timepoint, outputDir)

            slicer.util.showStatusMessage("Fiducials + metrics saved successfully!", 5000)

        except Exception as e:
            slicer.util.errorDisplay(f"Export failed: {e}")

    def onOpenInstructionsPDF(self):
        try:
            moduleDir = os.path.dirname(__file__)
            pdfPath = os.path.join(moduleDir, "Resources", "Guide_Placement.pdf")

            if not os.path.exists(pdfPath):
                slicer.util.errorDisplay("PDF file not found.")
                return

            qt.QDesktopServices.openUrl(qt.QUrl.fromLocalFile(pdfPath))

        except Exception as e:
            slicer.util.errorDisplay(f"Failed to open PDF: {str(e)}")

    def onCalculateMetrics(self, timepoint: str):
        """Compute all metrics from fiducials and draw them.

        Point numbering (15 fiducials total) and anatomical meaning:
            P1  - Diaphysis proximal medial
            P2  - Diaphysis proximal lateral
            P3  - Diaphysis distal medial
            P4  - Diaphysis distal lateral
            P5  - GT lateral angle
            P6  - Head-GT junction (placed on the glenosphere rim instead, for Reverse prosthesis X-rays)
            P7  - Head-calcar junction (placed on the glenosphere rim instead, for Reverse prosthesis X-rays)
            P8  - Head surface
            P9  - Medial fossa
            P10 - Lateral fossa
            P11 - Upper glenoid
            P12 - Lower glenoid
            P13 - Lateral acromion
            P14 - Medial acromion
            P15 - Medial part of the scapular neck

        All geometric formulas below keep the exact same anatomical logic as
        before; only the point indices have been remapped to match this new
        numbering (see mapping comment in setup()).

        X-ray type (self.xrayTypeSelector) controls which metrics are computed:
            - Native / Anatomic prosthesis: standard metric set only.
            - Reverse prosthesis: standard metric set PLUS two additional
              metrics that rely on the glenosphere axis (P6->P7):
                - GI  (Glenosphere Inclination): angle between the fossa axis
                  (P9->P10) and the glenosphere axis (P6->P7).
                - Overhang: signed distance from P4 to P12, projected along
                  the glenosphere axis (P6->P7).
        """
        nodeName = f"Fiducials_{timepoint}"
        outputDir = self.outputDirectoryButton.currentPath
        markupNode = slicer.mrmlScene.GetFirstNodeByName(nodeName)
        if not markupNode:
            slicer.util.errorDisplay(f"No fiducials placed for {timepoint}")
            return

        patientID = self.patientIdLineEdit.text.strip()
        if not patientID:
            slicer.util.errorDisplay("Please enter a patient ID.")
            return

        points = self.logic.get_fiducial_positions(markupNode)
        if len(points) < len(self.fiducialLabels):
            slicer.util.errorDisplay(f"Not enough fiducials (required: {len(self.fiducialLabels)}).")
            return
        for label in points:
            points[label]['z'] = 0

        # --- Points ---
        P1 = np.array([points['P1']['x'], points['P1']['y'], points['P1']['z']])
        P2 = np.array([points['P2']['x'], points['P2']['y'], points['P2']['z']])
        P3 = np.array([points['P3']['x'], points['P3']['y'], points['P3']['z']])
        P4 = np.array([points['P4']['x'], points['P4']['y'], points['P4']['z']])
        P5 = np.array([points['P5']['x'], points['P5']['y'], points['P5']['z']])
        P6 = np.array([points['P6']['x'], points['P6']['y'], points['P6']['z']])
        P7 = np.array([points['P7']['x'], points['P7']['y'], points['P7']['z']])
        P8 = np.array([points['P8']['x'], points['P8']['y'], points['P8']['z']])
        P9 = np.array([points['P9']['x'], points['P9']['y'], points['P9']['z']])
        P10 = np.array([points['P10']['x'], points['P10']['y'], points['P10']['z']])
        P11 = np.array([points['P11']['x'], points['P11']['y'], points['P11']['z']])
        P12 = np.array([points['P12']['x'], points['P12']['y'], points['P12']['z']])
        P13 = np.array([points['P13']['x'], points['P13']['y'], points['P13']['z']])
        P14 = np.array([points['P14']['x'], points['P14']['y'], points['P14']['z']])
        P15 = np.array([points['P15']['x'], points['P15']['y'], points['P15']['z']])

        # --- Beta angle ---
        # Old logic used fossa cortex line (old P1,P2) vs glenoid line (old P3,P4)
        # New mapping: old P1->P9, old P2->P10, old P3->P12, old P4->P11
        beta_angle = self.logic.compute_beta_angle({
            'P1': {'x': P9[0], 'y': P9[1]},
            'P2': {'x': P10[0], 'y': P10[1]},
            'P3': {'x': P12[0], 'y': P12[1]},
            'P4': {'x': P11[0], 'y': P11[1]},
        })
        # Compute intersection in XY plane
        x1, y1 = P9[:2]
        x2, y2 = P10[:2]
        x3, y3 = P12[:2]
        x4, y4 = P11[:2]

        denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if denom == 0:
            # Lines are parallel
            intersect = P10.copy()
        else:
            px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denom
            py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denom
            intersect = np.array([px, py, 0.0])

        # Draw arc with the intersection as vertex
      #  self.logic.draw_arc(intersect, P9, P11, beta_angle, name="BetaAngleArc", color=(1, 1, 0))

        # --- Critical Shoulder Angle (CSA) ---
        # Old: compute_angle(P4, P3, P6) -> glenoid upper, glenoid lower, lateral acromion
        # New mapping: old P4->P11, old P3->P12, old P6->P13
        csa_angle = self.logic.compute_angle(P11, P12, P13)
        self.logic.draw_line(P12, P11, color=(0.5, 0, 1), name="CSA")
        self.logic.draw_line(P12, P13, color=(0.5, 0, 1))

        # --- Subacromial space (SAS) ---
        # Old: circle on humeral head contour points (old P9, P10, P11)
        # New mapping: old P9->P6, old P10->P7, old P11->P8
        center, radius = self.logic.compute_circle_center(P6, P7, P8)

        if center is not None:
            self.logic.draw_circle(center, radius, name="HumeralHeadCircle")

            # Direction of acromial line
            # Old: old P6 (lateral acromion), old P7 (medial acromion)
            # New mapping: old P6->P13, old P7->P14
            AB = P14 - P13
            AB_norm = AB / np.linalg.norm(AB)

            # Normal vector (perpendicular)
            normal = np.array([-AB_norm[1], AB_norm[0], 0])

            # Signed distance from center to acromial line
            d = self.logic.distance_point_to_line(center, P13, P14)

            # --- Choose correct side for tangent ---
            vec = center - P13
            sign = np.sign(np.dot(vec, normal))

            # Tangent point on circle
            tangent_point = center - sign * radius * normal

            # Draw tangent line (parallel)
            tangent_start = tangent_point - AB_norm * 50
            tangent_end = tangent_point + AB_norm * 50
            self.logic.draw_line(tangent_start, tangent_end, color=(1, 1, 0), name="SAS_tangent")
            self.logic.draw_line(P13, P14, color=(1, 1, 0))

            sas_distance = abs(d - radius)

        # --- Glenoid Height (GH) ---
        # Old: norm(P4 - P3) -> glenoid upper, glenoid lower
        # New mapping: old P4->P11, old P3->P12
        gh_distance = np.linalg.norm(P11 - P12)
      #  self.logic.draw_line(P12, P11, color=(1, 0, 1), name="GH")

        # --- Scapular Neck Length (SNL) ---
        # Old: norm(P5 - P3) -> scapular neck medial, glenoid lower
        # New mapping: old P5->P15, old P3->P12
        snl_distance = np.linalg.norm(P15 - P12)

        # --- Scapular Neck Angle (SNA) ---
        # Old: compute_angle(P4, P3, P5) -> glenoid upper, glenoid lower, scapular neck medial
        # New mapping: old P4->P11, old P3->P12, old P5->P15
        sna_angle = self.logic.compute_angle(P11, P12, P15)
   #     self.logic.draw_line(P11, P12, color=(1, 0.5, 0), name="SNA")
   #     self.logic.draw_line(P12, P15, color=(1, 0.5, 0))


        # --- Glenoid inclination angle (GIA) ---
       # gia_angle = 180 - self.logic.compute_beta_angle({
       #     'P1': {'x': P9[0], 'y': P9[1]},
       #     'P2': {'x': P10[0], 'y': P10[1]},
       #     'P3': {'x': P12[0], 'y': P12[1]},
       #     'P4': {'x': P11[0], 'y': P11[1]}
       # })
       # self.logic.draw_line(P9, P10, color=(0.8, 0.2, 1), name="GIA")
       # self.logic.draw_line(P12, P11, color=(0.8, 0.2, 1))

        # --- Distalization shoulder angle (DSA) ---
        # Old: compute_angle(P8, P4, P6) -> lateral bone-cartilage junction (merged into old P9),
        #      glenoid upper, lateral acromion
        # New mapping: old P8->P6 (merged), old P4->P11, old P6->P13
        dsa_angle = self.logic.compute_angle(P6, P11, P13)
        self.logic.draw_line(P6, P11, color=(0.9, 0.4, 0.6), name="DSA")
        self.logic.draw_line(P11, P13, color=(0.9, 0.4, 0.6))

        # --- Lateralization shoulder angle (LSA) ---
        # Old: compute_angle(P12, P6, P4) -> GT lateral edge, lateral acromion, glenoid upper
        # New mapping: old P12->P5, old P6->P13, old P4->P11
        lsa_angle = self.logic.compute_angle(P5, P13, P11)
        self.logic.draw_line(P5, P13, color=(0.9, 0.4, 0.6), name="LSA")
        self.logic.draw_line(P13, P11, color=(0.9, 0.4, 0.6))

        # --- Humeral Axis ---
        # Old: mid(P13,P14) to mid(P15,P16) -> diaphysis proximal/distal internal cortex
        # New mapping: old P13->P1, old P14->P2, old P15->P3, old P16->P4
        mid1 = (P1 + P2) / 2
        mid2 = (P3 + P4) / 2
        self.logic.draw_line(mid1, mid2, color=(1, 1, 0), name="HumeralAxis")
        axis_vector = mid2 - mid1
        axis_unit = axis_vector / np.linalg.norm(axis_vector)
        axis_length = np.linalg.norm(mid2 - mid1)

        # --- Reverse-prosthesis-only metrics ---
        # Only computed when the selected X-ray type is "Reverse prosthesis".
        # For these X-rays, P6/P7 are placed on the glenosphere rim instead of
        # the native humeral head contour, so P6->P7 represents the
        # glenosphere (GS) axis.
        xrayType = self.xrayTypeSelector.currentText if hasattr(self, "xrayTypeSelector") else "Native"
        is_reverse = (xrayType == "Reverse prosthesis")

        gi_angle = None
        overhang_distance = None

        if is_reverse:
            # --- Glenosphere Inclination (GI) ---
            # Angle between the fossa axis (P9->P10) and the glenosphere axis (P6->P7).
            gi_angle = self.logic.compute_angle_between_vectors(P10 - P9, P7 - P6)
            self.logic.draw_line(P9, P10, color=(0, 1, 1), name="GI_fossa_axis")
            self.logic.draw_line(P6, P7, color=(0, 1, 1), name="GI_GS_axis")

            # --- Overhang ---
            # Signed distance from P4 to P12, projected along the glenosphere axis (P6->P7).
            overhang_distance = self.logic.project_vector_onto_axis(P4 - P12, P6, P7)

        # Save metrics
        metrics = {
            "beta_angle": beta_angle,
            "critical_shoulder_angle": csa_angle,
            "subacromial_space": sas_distance,
            "GH": gh_distance,
            "SNL": snl_distance,
            "SNA": sna_angle,
         #   "GIA": gia_angle,
            "DSA": dsa_angle,
            "LSA": lsa_angle,
            "HumeralAxis": axis_vector
        }
        if is_reverse:
            metrics["GI"] = gi_angle
            metrics["Overhang"] = overhang_distance
       # self.logic.saveCombinedCSV(markupNode, metrics, patientID, timepoint, outputDir)
        self.currentMetrics = metrics
        # --- Format results as table ---
        text = (
            f"Patient: {patientID}\n"
            f"-------------------------\n"
            f"Beta : {beta_angle:.1f}°\n"
            f"CSA  : {csa_angle:.1f}°\n"
            f"SAS  : {sas_distance:.1f} mm\n"
            f"GH   : {gh_distance:.1f} mm\n"
            f"SNL  : {snl_distance:.1f} mm\n"
            f"SNA  : {sna_angle:.1f}°\n"
        #    f"GIA  : {gia_angle:.1f}°\n"
            f"DSA  : {dsa_angle:.1f}°\n"
            f"LSA  : {lsa_angle:.1f}°\n"
        )
        if is_reverse:
            text += (
                f"GI   : {gi_angle:.1f}°\n"
                f"Overhang : {overhang_distance:.1f} mm\n"
            )


        # --- Display in slice views ---
        layoutManager = slicer.app.layoutManager()

        for viewName in ["Red", "Yellow", "Green"]:
            sliceWidget = layoutManager.sliceWidget(viewName)
            if sliceWidget:
                sliceView = sliceWidget.sliceView()
                cornerAnnotation = sliceView.cornerAnnotation()
                cornerAnnotation.SetText(vtk.vtkCornerAnnotation.UpperLeft, text)
                sliceView.render()

        # --- FORCE HIDE LABELS ---

    def onToggleFiducialLabels(self):
        """Toggle fiducial labels visibility."""
        # Check first node to decide current state
        scene = slicer.mrmlScene
        fidNodes = scene.GetNodesByClass("vtkMRMLMarkupsFiducialNode")
        if not fidNodes:
            slicer.util.showStatusMessage("No fiducial nodes in the scene.", 2000)
            return

        displayNode = fidNodes.GetItemAsObject(0).GetDisplayNode()
        if not displayNode:
            return

        currentVisibility = displayNode.GetPointLabelsVisibility()
        newVisibility = not currentVisibility

        self.setFiducialLabelsVisible(newVisibility)

        if newVisibility:
            slicer.util.showStatusMessage("Fiducial labels shown.", 2000)
        else:
            slicer.util.showStatusMessage("Fiducial labels hidden.", 2000)

    def setFiducialLabelsVisible(self, visible: bool = True):
        """Show or hide all fiducial labels in the scene."""
        scene = slicer.mrmlScene
        for node in scene.GetNodesByClass("vtkMRMLMarkupsFiducialNode"):
            displayNode = node.GetDisplayNode()
            if displayNode:
                displayNode.SetPointLabelsVisibility(visible)
                displayNode.SetPropertiesLabelVisibility(visible)
                node.Modified()

class Beta_angle_anaLogic(ScriptedLoadableModuleLogic):
    """Logic class for data processing, angle computation, and visualization."""

    def __init__(self):
        super().__init__()

    # --- Fiducial retrieval ---
    def get_fiducial_positions(self, markupNode):
        """Return a dict of fiducial positions keyed by label {P1: {x,y,z}, ...}."""
        points = {}
        if not markupNode:
            return points
        for i in range(markupNode.GetNumberOfControlPoints()):
            label = markupNode.GetNthControlPointLabel(i)
            pos = [0.0, 0.0, 0.0]
            markupNode.GetNthControlPointPosition(i, pos)
            points[label] = {"x": pos[0], "y": pos[1], "z": pos[2]}
        return points

    # --- Save fiducials + metrics together in a single CSV file ---
    def saveCombinedCSV(self, markupNode, metrics, patientID, timepoint, outputDir):
        """
        Write a single CSV file containing two sections:
          1) FIDUCIALS  - one row per fiducial point (ID, Label, X, Y, Z)
          2) PARAMETERS - one row per computed metric (Metric, Value)
        The two sections are separated by a blank line and a section header,
        acting as a lightweight substitute for two Excel sheets.
        """
        if not os.path.exists(outputDir):
            slicer.util.errorDisplay("Selected output folder does not exist.")
            return

        fileName = f"{patientID}_{timepoint}_combined.csv"
        filePath = os.path.join(outputDir, fileName)

        try:
            points = self.get_fiducial_positions(markupNode)

            with open(filePath, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)

                # --- Section 1: Fiducials ---
                writer.writerow(["FIDUCIALS"])
                writer.writerow(["ID", "Label", "X", "Y", "Z"])
                for i in range(markupNode.GetNumberOfControlPoints()):
                    label = markupNode.GetNthControlPointLabel(i)
                    pos = points.get(label, {"x": "", "y": "", "z": ""})
                    writer.writerow([i + 1, label, pos["x"], pos["y"], pos["z"]])

                # --- Blank line between sections ---
                writer.writerow([])

                # --- Section 2: Parameters ---
                writer.writerow(["PARAMETERS"])
                writer.writerow(["Metric", "Value"])
                for key, value in metrics.items():
                    if isinstance(value, (list, tuple, np.ndarray)):
                        value_str = ";".join(str(v) for v in np.array(value).flatten())
                    else:
                        value_str = str(value)
                    writer.writerow([key, value_str])

            slicer.util.showStatusMessage(f"Fiducials + metrics saved to: {filePath}", 5000)

        except PermissionError:
            slicer.util.errorDisplay("Permission denied. Please choose another folder.")
        except Exception as e:
            slicer.util.errorDisplay(f"Failed to save combined CSV: {e}")



    # --- Compute beta angle ---
    def compute_beta_angle(self, points):
        """Compute Beta angle (P1-P2 vs P3-P4)."""
        p1 = np.array([points['P1']['x'], points['P1']['y']])
        p2 = np.array([points['P2']['x'], points['P2']['y']])
        p3 = np.array([points['P3']['x'], points['P3']['y']])
        p4 = np.array([points['P4']['x'], points['P4']['y']])

        v1 = p2 - p1
        v2 = p4 - p3
        v1_norm = v1 / np.linalg.norm(v1)
        v2_norm = v2 / np.linalg.norm(v2)
        dot = np.clip(np.dot(v1_norm, v2_norm), -1.0, 1.0)
        return 180 - np.degrees(np.arccos(dot))

    # --- Generic angle computation ---
    def compute_angle(self, A, B, C):
        """Compute angle at B formed by A-B-C."""
        BA = A - B
        BC = C - B
        BA_norm = BA / np.linalg.norm(BA)
        BC_norm = BC / np.linalg.norm(BC)
        dot = np.clip(np.dot(BA_norm, BC_norm), -1.0, 1.0)
        return np.degrees(np.arccos(dot))

    # --- Angle between two free vectors (not sharing a common vertex point) ---
    def compute_angle_between_vectors(self, v1, v2):
        """
        Compute the angle (in degrees, 0-90) between two direction vectors,
        e.g. between an axis defined by two points and another axis defined
        by two other points. Unlike compute_angle, this does not require a
        shared vertex - each axis is given directly as a vector.

        Used for Glenosphere Inclination (GI): angle between the fossa axis
        (P9->P10) and the glenosphere axis (P6->P7).
        """
        v1_norm = v1 / np.linalg.norm(v1)
        v2_norm = v2 / np.linalg.norm(v2)
        dot = np.clip(np.dot(v1_norm, v2_norm), -1.0, 1.0)
        angle = np.degrees(np.arccos(dot))
        # Report the acute angle between the two lines (0-90 deg), since an
        # axis has no inherent direction and the obtuse/acute angle depends
        # only on arbitrary point placement order.
        return min(angle, 180 - angle)

    # --- Signed projection of a vector onto an axis defined by two points ---
    def project_vector_onto_axis(self, vec, axisA, axisB):
        """
        Project `vec` onto the axis defined by (axisA -> axisB) and return
        the signed length of that projection.

        Used for Overhang: signed distance from P4 to P12 (vec = P4 - P12),
        projected along the glenosphere axis (P6->P7, axisA=P6, axisB=P7).
        """
        axis_vec = axisB - axisA
        axis_len = np.linalg.norm(axis_vec)
        if axis_len == 0:
            return 0.0
        axis_unit = axis_vec / axis_len
        return float(np.dot(vec, axis_unit))

    # --- Distance point-to-line ---
    def distance_point_to_line(self, P, A, B):
        """Distance from P to line A-B."""
        line_vec = B - A
        point_vec = P - A
        line_len = np.linalg.norm(line_vec)
        if line_len == 0:
            return 0, A
        proj = np.dot(point_vec, line_vec) / line_len
        closest = A + proj * line_vec / line_len
        dist = np.linalg.norm(P - closest)
        return dist #, closest


    # --- Generate points along an arc ---
    def generate_arc_points(self, vertex, p1, p2, angle_deg, num_points=30):
        """
        Generate points along an arc centered at `vertex`, starting at `p1` and spanning `angle_deg` toward `p2`.
        """
        v1 = p1 - vertex
        radius = np.linalg.norm(v1)
        angle_rad = np.radians(angle_deg)

        points = []
        for t in np.linspace(0, 1, num_points):
            theta = t * angle_rad
            # 2D rotation in XY plane
            rot_matrix = np.array([[np.cos(theta), -np.sin(theta)],
                                   [np.sin(theta), np.cos(theta)]])
            arc_point_2d = vertex[:2] + rot_matrix @ v1[:2]
            points.append([arc_point_2d[0], arc_point_2d[1], vertex[2]])
        return points

    # --- Draw arc ---
    def draw_arc(self, vertex, p1, p2, angle_deg, name="Arc", color=(1, 1, 0), thickness=0.00001, num_points=30):
        """
        Draw an arc centered at `vertex`, starting at `p1` and spanning `angle_deg` toward `p2`.
        The arc is drawn in the XY plane.
        """
        # Remove old node with the same name
        for node in slicer.mrmlScene.GetNodesByClass("vtkMRMLMarkupsCurveNode"):
            if node.GetName() == name:
                slicer.mrmlScene.RemoveNode(node)

        # Generate points along the arc
        v_start = p1 - vertex
        v_end = p2 - vertex

        # Determine rotation direction (sign of angle)
        angle_rad = np.radians(angle_deg)
        points = []
        for t in np.linspace(0, 1, num_points):
            theta = t * angle_rad
            # 2D rotation in XY plane
            rot_matrix = np.array([[np.cos(theta), -np.sin(theta)],
                                   [np.sin(theta), np.cos(theta)]])
            arc_point_xy = vertex[:2] + rot_matrix @ v_start[:2]
            points.append([arc_point_xy[0], arc_point_xy[1], vertex[2]])

        # Create Markups Curve Node
        curveNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsCurveNode", name)
        for pt in points:
            curveNode.AddControlPoint(pt)

        # Display properties
        displayNode = curveNode.GetDisplayNode()

        if displayNode:
            displayNode.SetSelectedColor(*color)
            displayNode.SetLineThickness(thickness)
            displayNode.SetVisibility2D(True)
            displayNode.SetVisibility3D(True)

    # --- Draw line ---
    def draw_line(self, A, B, color=(1, 0, 0), thickness=0.2, name = None):
        """Draw a line as a Markups Line Node."""
        # Remove previous line node with same name
        for node in slicer.mrmlScene.GetNodesByClass("vtkMRMLMarkupsLineNode"):
            if node.GetName() == name:
                slicer.mrmlScene.RemoveNode(node)

        lineNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsLineNode") #, name
        lineNode.AddControlPoint(A.tolist())
        lineNode.AddControlPoint(B.tolist())

        displayNode = lineNode.GetDisplayNode()

        if displayNode:
            displayNode.SetSelectedColor(*color)
            displayNode.SetLineThickness(thickness)
            displayNode.SetVisibility2D(True)
            displayNode.SetVisibility3D(True)

            displayNode.SetPointLabelsVisibility(False)
            displayNode.SetPropertiesLabelVisibility(False)

            lineNode.SetName("")

            lineNode.SetNthControlPointLabel(0, "")
            lineNode.SetNthControlPointLabel(1, "")

            if name:
                lineNode.SetName(name)

                # show only the custom label
                displayNode.SetPointLabelsVisibility(True)
                displayNode.SetPropertiesLabelVisibility(False)

                lineNode.SetNthControlPointLabel(0, name)
                lineNode.SetNthControlPointLabel(1, "")

    # --- Calculate the center of the circle with 3 reference point to find the center of rotation of the humeral head ---
    def compute_circle_center(self, A, B, C):
        """
        Compute circle center passing through 3 points (2D).
        """
        ax, ay = A[:2]
        bx, by = B[:2]
        cx, cy = C[:2]

        d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))

        if abs(d) < 1e-6:
            return None, None

        ux = ((ax ** 2 + ay ** 2) * (by - cy) +
              (bx ** 2 + by ** 2) * (cy - ay) +
              (cx ** 2 + cy ** 2) * (ay - by)) / d

        uy = ((ax ** 2 + ay ** 2) * (cx - bx) +
              (bx ** 2 + by ** 2) * (ax - cx) +
              (cx ** 2 + cy ** 2) * (bx - ax)) / d

        center = np.array([ux, uy, A[2]])
        radius = np.linalg.norm(center - A)

        return center, radius

    def draw_circle(self, center, radius, name="Circle", color=(0, 1, 1), num_points=18):

        import numpy as np

        for node in slicer.mrmlScene.GetNodesByClass("vtkMRMLMarkupsCurveNode"):
            if node.GetName() == name:
                slicer.mrmlScene.RemoveNode(node)

        curveNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsCurveNode", name)

        for t in np.linspace(0, 2 * np.pi, num_points):
            x = center[0] + radius * np.cos(t)
            y = center[1] + radius * np.sin(t)
            curveNode.AddControlPoint([x, y, center[2]])

        displayNode = curveNode.GetDisplayNode()
        displayNode.SetSelectedColor(*color)
        displayNode.SetLineThickness(0.002)

    # --- Compute the tangent to a circle ---
    def compute_tangent_at_point(self, center, point):
        """
        Compute tangent line at a point of a circle.
        Returns two points defining the tangent.
        """

        radius_vec = point - center

        # perpendicular vector (2D)
        tangent_vec = np.array([-radius_vec[1], radius_vec[0], 0])

        tangent_vec = tangent_vec / np.linalg.norm(tangent_vec)

        p1 = point - 50 * tangent_vec
        p2 = point + 50 * tangent_vec

        return p1, p2

    def draw_parallel_tangent(self, center, radius, A, B, name="SAS_Tangent"):

        AB = B - A

        normal = np.array([-AB[1], AB[0], 0])
        normal = normal / np.linalg.norm(normal)

        tangent_point = center + radius * normal

        p1 = tangent_point - 50 * AB / np.linalg.norm(AB)
        p2 = tangent_point + 50 * AB / np.linalg.norm(AB)

        self.draw_line(p1, p2, name=name, color=(0, 0, 1))