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

    def __init__(self, parent):
        ScriptedLoadableModule.__init__(self, parent)
        self.parent.title = _("Beta_angle_ana")
        # Keep categories concise and correct
        self.parent.categories = [ "Custome_model"]
        self.parent.dependencies = []
        self.parent.contributors = ["Elise Joulain (UNIGE)"]
        self.parent.helpText = _("""
This module assists with placing fiducials on shoulder X-rays and computing various
metrics (beta angle and related).
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
    inputVolume: vtkMRMLScalarVolumeNode
    imageThreshold: Annotated[float, WithinRange(-100, 500)] = 100
    invertThreshold: bool = False
    thresholdedVolume: vtkMRMLScalarVolumeNode
    invertedVolume: vtkMRMLScalarVolumeNode


class Beta_angle_anaWidget(ScriptedLoadableModuleWidget, VTKObservationMixin):
    def __init__(self, parent=None) -> None:
        ScriptedLoadableModuleWidget.__init__(self, parent)
        VTKObservationMixin.__init__(self)
        self.logic = None
        self.inputVolumeSelector = None
        self.markupNode = None

    def setup(self) -> None:
        ScriptedLoadableModuleWidget.setup(self)

        self.fiducialLabels = {
            "P1": "Diaphysis proximal medial - external border of cortex",
            "P2": "Diaphysis proximal lateral - external border of cortex",
            "P3": "Diaphysis distal medial - external border of cortex",
            "P4": "Diaphysis distal lateral - external border of cortex",
            "P5": "Diaphysis mid medial - external cortex",
            "P6": "Diaphysis mid lateral - external cortex",
            "P7": "Latero-superior greater tuberosity (GT)",
            "P8": "Upper head greater tuberosity junction",
            "P9": "Lower head calcar junction",
            "P10": "Mid head calcar surface",
            "P11": "Lateral fossa ",
            "P12": "Medial fossa",
            "P13": "Mid fossa",
            "P14": "Upper glenoid",
            "P15": "Lower glenoid",
            "P16": "Lateroinferior acromion",
            "P17": "Medio-inferior acromion",
            "P18": "Lower cortical edge of acromion (for circle fit)",
            "P19": "Medial part of the scapular neck",
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
        self.resetFiducialsButton.connect('clicked(bool)', self.onResetFiducials)
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

    def onResetFiducials(self):
        scene = slicer.mrmlScene

        if not slicer.util.confirmYesNoDisplay("Are you sure you want to remove all fiducial nodes?"):
            return

        for node in list(scene.GetNodesByClass("vtkMRMLMarkupsFiducialNode")):
            name = node.GetName() or ""
            if "beta_angle" in name.lower():
                scene.RemoveNode(node)
        for node in scene.GetNodesByClass("vtkMRMLMarkupsLineNode"):
            scene.RemoveNode(node)
        for node in scene.GetNodesByClass("vtkMRMLMarkupsCurveNode"):
            scene.RemoveNode(node)

    def onPlaceFiducial(self, timepoint: str):
        """Activate placement of fiducials and create the markup node if needed."""
        nodeName = f"Fiducials_{timepoint}"
        self.markupNode = slicer.mrmlScene.GetFirstNodeByName(nodeName)
        if not self.markupNode:
            self.markupNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode", nodeName)
            self.markupNode.AddObserver(slicer.vtkMRMLMarkupsNode.PointAddedEvent, self.onFiducialAdded)

        if "beta_angle" in timepoint.lower():
            displayNode = self.markupNode.GetDisplayNode()
            if displayNode:
                displayNode.SetSelectedColor(0, 1, 0)  #
        slicer.modules.markups.logic().StartPlaceMode(1)

    def onFiducialAdded(self, caller, event):
        """Automatically label new fiducials P1..PN up to the number of labels provided."""
        n = caller.GetNumberOfControlPoints()
        if n <= len(self.fiducialLabels):
            fidName = f"P{n}"
            caller.SetNthControlPointLabel(n - 1, fidName)

            if fidName == "P19":
                self.drawGuideLineForP19(caller)
        else:
            # Stop placement if too many points
            interactionNode = slicer.app.applicationLogic().GetInteractionNode()
            interactionNode.SetCurrentInteractionMode(slicer.vtkMRMLInteractionNode.ViewTransform)
            slicer.util.showStatusMessage("Maximum fiducials placed — placement stopped.", 3000)

    def drawGuideLineForP19(self, markupNode, half_length=80.0):
        """
        Draw a temporary guide line through P12, parallel to the fossa axis
        (TLS fit through P9, P10, P11), to help the user place P16 along the
        expected direction of the scapular neck.
        """
        try:
            points = self.logic.get_fiducial_positions(markupNode)
            required = ("P11", "P12", "P13", "P14")
            missing = [p for p in required if p not in points]
            if missing:
                logging.warning("P18 guide line skipped: missing %s", missing)
                return

            P11 = np.array([points['P11']['x'], points['P11']['y'], 0.0])
            P12 = np.array([points['P12']['x'], points['P12']['y'], 0.0])
            P13 = np.array([points['P13']['x'], points['P13']['y'], 0.0])
            P14 = np.array([points['P14']['x'], points['P14']['y'], 0.0])

            _, fossa_dir = self.logic.fit_line_tls([P11, P12, P14])
            if np.dot(fossa_dir, P12 - P11) < 0:
                fossa_dir = -fossa_dir

            guide_start = P14 - half_length * fossa_dir
            guide_end = P14 + half_length * fossa_dir
            self.logic.draw_line(guide_start, guide_end, color=(0, 1, 0),
                                 name="P19_GuideLine") #locked=True
            logging.info("P19 guide line drawn from %s to %s", guide_start, guide_end)


            selectionNode = slicer.app.applicationLogic().GetSelectionNode()
            selectionNode.SetActivePlaceNodeClassName(markupNode.GetClassName())
            selectionNode.SetActivePlaceNodeID(markupNode.GetID())
            slicer.modules.markups.logic().StartPlaceMode(1)

        except Exception:
            logging.exception("drawFossaGuideLineForP19 failed")

############## to delete ?
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
        P16 = np.array([points['P16']['x'], points['P16']['y'], points['P16']['z']])
        P17 = np.array([points['P17']['x'], points['P17']['y'], points['P17']['z']])
        P18 = np.array([points['P18']['x'], points['P18']['y'], points['P18']['z']])
        P19 = np.array([points['P19']['x'], points['P19']['y'], points['P19']['z']])

        # --- Fossa axis (TLS fit through 3 points) ---
        fossa_centroid, fossa_dir = self.logic.fit_line_tls([P9, P10, P11])
        if np.dot(fossa_dir, P10 - P9) < 0:
            fossa_dir = -fossa_dir
        #self.logic.draw_line(fossa_dir, color=(0.9, 0.4, 0.6), name="Fossa axis")

        # --- Beta angle ---
        # fossa axis (fit from P9, P10, P11) vs glenoid line (lower P13 -> upper P12)
        beta_angle = self.logic.compute_beta_angle_from_vectors(fossa_dir, P12 - P13)

        # Vertex for the arc drawing: intersection of the raw P9-P10 fossa segment with the glenoid line
        x1, y1 = P11[:2]
        x2, y2 = P12[:2]
        x3, y3 = P15[:2]
        x4, y4 = P14[:2]

        denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if denom == 0:
            intersect = P10.copy()
        else:
            px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denom
            py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denom
            intersect = np.array([px, py, 0.0])

        self.logic.draw_arc(intersect, P9, P12, beta_angle, name="BetaAngleArc", color=(1, 1, 0), num_points=5)


        # --- Critical Shoulder Angle (CSA) ---

        csa_angle = self.logic.compute_angle(P14, P15, P16)
        #self.logic.draw_line(P15, P14, color=(0.5, 0, 1), name="CSA")
        #self.logic.draw_line(P15, P16, color=(0.5, 0, 1))

        # --- Subacromial space (SAS) ---
        head_center, head_radius = self.logic.compute_circle_center(P8, P9, P10)
            # Acromion circle
        acr_center, acr_radius = self.logic.compute_circle_center(P16, P17, P18)

        sas_distance = None
        if head_center is not None and acr_center is not None:
            self.logic.draw_circle(head_center, head_radius, name="HumeralHeadCircle")
          #  self.logic.draw_circle(acr_center, acr_radius, name="AcromionCircle", color=(1, 0.5, 0))

            sas_distance, (acr_pt, head_pt), arc_pts = self.logic.compute_sas_circle_to_circle(
                head_center, head_radius, acr_center, acr_radius, P16, P17, P18
            )
           # self.logic.draw_arc_points(arc_pts, name="AcromionArcUsed", color=(1, 0.5, 0))
            self.logic.draw_line(acr_pt, head_pt, color=(1, 1, 0), name="SAS_distance")
        else:
            logging.warning("SAS not computed: head and/or acromion points "
                            "are collinear - check P8-P10 and P16-P18 placement.")

            # --- Glenoid Height (GH) ---
            gh_distance = np.linalg.norm(P14 - P15)

            # --- Scapular Neck Length (SNL) ---
            snl_distance = np.linalg.norm(P19 - P15)

            # --- Scapular Neck Angle (SNA) ---
            sna_angle = self.logic.compute_angle(P14, P15, P19)

            # --- Distalization shoulder angle (DSA) ---
            dsa_angle = self.logic.compute_angle(P8, P14, P16)

            # --- Lateralization shoulder angle (LSA) ---
            lsa_angle = self.logic.compute_angle(P7, P16, P14)

        # --- Humeral Axis ---
        # P1, P3, P5 = inner cortical edge; P2, P4, P6 = outer cortical edge
        res = self.logic.central_axis_from_cortices([P1, P3, P5], [P2, P4, P6])

        mid1 = res["p_start"]
        mid2 = res["p_end"]
        self.logic.draw_line(mid1, mid2, color=(1, 1, 0), name="HumeralAxis")

        axis_unit = res["direction"]
        axis_vector = mid2 - mid1
        axis_length = np.linalg.norm(axis_vector)

        logging.info("Humeral axis: %d iter | half-width %.2f mm | "
                     "asymmetry %.4f mm",
                     res["n_iter"], res["half_width_mean"], res["asymmetry_rms"])

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
            gi_angle = self.logic.compute_angle_between_vectors(fossa_dir, P9 - P8)

            # draw the fitted fossa line across the extent of all 3 fossa points, not just P9-P10
            fossa_pts = np.vstack([P11, P12, P13])
            t_fossa = (fossa_pts - fossa_centroid) @ fossa_dir
            fossa_line_start = fossa_centroid + t_fossa.min() * fossa_dir
            fossa_line_end = fossa_centroid + t_fossa.max() * fossa_dir
            self.logic.draw_line(fossa_line_start, fossa_line_end, color=(0, 1, 1), name="GI_fossa_axis")
            self.logic.draw_line(P8, P9, color=(0, 1, 1), name="GI_GS_axis")

            # --- Overhang ---
            overhang_distance = self.logic.project_vector_onto_axis(P4 - P13, P6, P7)

        # Save metrics
        metrics = {
            "beta_angle": beta_angle,
            "critical_shoulder_angle": csa_angle,
            "subacromial_space": sas_distance,
            "GH": gh_distance,
            "SNL": snl_distance,
            "SNA": sna_angle,
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
            f"DSA  : {dsa_angle:.1f}°\n"
            f"LSA  : {lsa_angle:.1f}°\n"
            f"HumeralAxis  : {axis_vector:.1f}°\n"
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
        The two sections are separated by a blank line and a section header.
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



    # --- Compute beta angle --- TO DELETE
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

    # --- Beta angle from two direction vectors (used when one line is a multi-point fit) ---
    def compute_beta_angle_from_vectors(self, v1, v2):
        v1n = v1 / np.linalg.norm(v1)
        v2n = v2 / np.linalg.norm(v2)
        dot = np.clip(np.dot(v1n, v2n), -1.0, 1.0)
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

    # --- Angle between two free vectors  ---
    def compute_angle_between_vectors(self, v1, v2):
        """
        Compute the angle (in degrees) between two direction vectors.
        """
        v1_norm = v1 / np.linalg.norm(v1)
        v2_norm = v2 / np.linalg.norm(v2)
        dot = np.clip(np.dot(v1_norm, v2_norm), -1.0, 1.0)
        angle = np.degrees(np.arccos(dot))

        return min(angle, 180 - angle)

    # --- Signed projection of a vector onto an axis defined by two points ---
    def project_vector_onto_axis(self, vec, axisA, axisB):
        """
        Project `vec` onto the axis defined by (axisA -> axisB) and return
        the signed length of that projection.

        Used for Overhang
        """
        axis_vec = axisB - axisA
        axis_len = np.linalg.norm(axis_vec)
        if axis_len == 0:
            return 0.0
        axis_unit = axis_vec / axis_len
        return float(np.dot(vec, axis_unit))

    # --- Compute least-squares line fit ---
    def fit_line_tls(self, points):
        """Total least-squares line fit. Returns (centroid, unit_direction)."""
        pts = np.asarray(points, dtype=float)
        centroid = pts.mean(axis=0)
        _, _, vt = np.linalg.svd(pts - centroid, full_matrices=False)
        direction = vt[0]
        return centroid, direction / np.linalg.norm(direction)

    # --- Compute the diaphysis axis running midway between the two cortical lines.
    def central_axis_from_cortices(self, inner_pts, outer_pts, n_samples=64,
                                   max_iter=30, tol=1e-12, extend=0.0):
        """
        Diaphyseal axis running midway between the two cortical lines.

        The direction converges to the angle bisector of the two cortical
        lines: the unique axis equally inclined to both edges.
        """
        inner_pts = np.asarray(inner_pts, dtype=float)
        outer_pts = np.asarray(outer_pts, dtype=float)

        c_in, u_in = self.fit_line_tls(inner_pts)
        c_out, u_out = self.fit_line_tls(outer_pts)
        if np.dot(u_in, u_out) < 0.0:
            u_out = -u_out

        all_pts = np.vstack([inner_pts, outer_pts])
        origin = all_pts.mean(axis=0)
        axis = u_in + u_out
        axis /= np.linalg.norm(axis)

        mids = p_in = p_out = None
        n_iter = 0

        for n_iter in range(1, max_iter + 1):
            h_all = (all_pts - origin) @ axis
            h_lo, h_hi = h_all.min(), h_all.max()

            d_in, d_out = np.dot(u_in, axis), np.dot(u_out, axis)
            if abs(d_in) < 1e-6 or abs(d_out) < 1e-6:
                raise ValueError("A cortical line is nearly perpendicular to the "
                                 "axis - check the inner/outer point assignment.")

            heights = np.linspace(h_lo, h_hi, n_samples)
            t_in = (heights - np.dot(c_in - origin, axis)) / d_in
            t_out = (heights - np.dot(c_out - origin, axis)) / d_out
            p_in = c_in + t_in[:, None] * u_in
            p_out = c_out + t_out[:, None] * u_out
            mids = 0.5 * (p_in + p_out)

            new_origin, new_axis = self.fit_line_tls(mids)
            if np.dot(new_axis, axis) < 0.0:
                new_axis = -new_axis

            converged = (1.0 - abs(np.dot(new_axis, axis))) < tol
            origin, axis = new_origin, new_axis
            if converged:
                break

        t = (mids - origin) @ axis
        p_start = origin + (t.min() - extend) * axis
        p_end = origin + (t.max() + extend) * axis

        def perp_dist(pts):
            v = pts - origin
            return np.linalg.norm(v - np.outer(v @ axis, axis), axis=1)

        r_in, r_out = perp_dist(p_in), perp_dist(p_out)

        return {
            "origin": origin,
            "direction": axis,
            "p_start": p_start,
            "p_end": p_end,
            "half_width_mean": float(0.5 * (r_in.mean() + r_out.mean())),
            "asymmetry_rms": float(np.sqrt(np.mean((r_in - r_out) ** 2))),
            "n_iter": n_iter,
        }

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
    def draw_arc(self, vertex, p1, p2, angle_deg, name="Arc", color=(1, 1, 0), thickness=0.0000001, num_points=30):
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

        angle_rad = np.radians(angle_deg)
        points = []
        for t in np.linspace(0, 1, num_points):
            theta = t * angle_rad
            rot_matrix = np.array([[np.cos(theta), -np.sin(theta)],
                                   [np.sin(theta), np.cos(theta)]])
            arc_point_xy = vertex[:2] + rot_matrix @ v_start[:2]
            points.append([arc_point_xy[0], arc_point_xy[1], vertex[2]])

        # Create Markups Curve Node
        curveNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsCurveNode", name)
        for pt in points:
            curveNode.AddControlPoint(pt)

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
        #lineNode.SetLocked(locked)

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

    # --- Sample points along the arc of a circle that runs between two
    #     boundary points, through a third point that disambiguates which
    #     of the two possible arcs (minor/major) is the right one. ---
    def circle_arc_points(self, center, radius, p_start, p_end, num_points=200):
        def angle_of(p):
            return np.arctan2(p[1] - center[1], p[0] - center[0])

        a_start = angle_of(p_start)
        a_end = angle_of(p_end)
        d_end = (a_end - a_start) % (2 * np.pi)  # CCW distance, in [0, 2pi)

        if d_end <= np.pi:
            thetas = a_start + np.linspace(0, d_end, num_points)
        else:
            thetas = a_start - np.linspace(0, 2 * np.pi - d_end, num_points)

        x = center[0] + radius * np.cos(thetas)
        y = center[1] + radius * np.sin(thetas)
        z = np.full_like(thetas, center[2])
        return np.stack([x, y, z], axis=1)


    # --- Distance from a point to the nearest point on a circle (2D, in-plane) ---
    def point_to_circle_distance(self, point, center, radius):
        v = point[:2] - center[:2]
        n = np.linalg.norm(v)
        if n < 1e-9:
            closest_xy = center[:2] + np.array([radius, 0.0])
        else:
            closest_xy = center[:2] + radius * v / n
        closest = np.array([closest_xy[0], closest_xy[1], center[2]])
        dist = np.linalg.norm(point - closest)
        return dist, closest

    # --- Subacromial space (SAS): minimal distance between the acromion
    #     circle (restricted to its P16->P17 arc, through P18) and the
    #     full humeral head circle. ---
    def compute_sas_circle_to_circle(self, head_center, head_radius,
                                     acr_center, acr_radius,
                                     p16, p17, p18, num_samples=200):
        arc_pts = self.circle_arc_points(acr_center, acr_radius, p16, p17,
                                         num_points=num_samples)

        best_dist, best_pair = None, None
        for pt in arc_pts:
            dist, closest_on_head = self.point_to_circle_distance(pt, head_center, head_radius)
            if best_dist is None or dist < best_dist:
                best_dist = dist
                best_pair = (pt, closest_on_head)

        return best_dist, best_pair, arc_pts

    # --- Draw an explicit list of points as a curve (used to show which
    #     arc of the acromion circle was actually searched) ---
    def draw_arc_points(self, points, name="Arc", color=(1, 0.5, 0), thickness=0.3):
        for node in slicer.mrmlScene.GetNodesByClass("vtkMRMLMarkupsCurveNode"):
            if node.GetName() == name:
                slicer.mrmlScene.RemoveNode(node)

        curveNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsCurveNode", name)
        for pt in points:
            curveNode.AddControlPoint(pt.tolist())

        displayNode = curveNode.GetDisplayNode()
        if displayNode:
            displayNode.SetSelectedColor(*color)
            displayNode.SetLineThickness(thickness)

    def draw_circle(self, center, radius, name="Circle", color=(0, 1, 1), num_points=18):

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