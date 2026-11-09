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
        self.parent.categories = ["Custome_model"]
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


def _fmt(value, unit=""):
    """Safe display of a metric: None -> N/A, arrays -> (x, y, z)."""
    if value is None:
        return "N/A"
    if isinstance(value, np.ndarray):
        if value.size == 1:
            return f"{float(value):.1f}{unit}"
        return "(" + ", ".join(f"{v:.1f}" for v in value) + f"){unit}"
    return f"{value:.1f}{unit}"


class Beta_angle_anaWidget(ScriptedLoadableModuleWidget, VTKObservationMixin):
    def __init__(self, parent=None) -> None:
        ScriptedLoadableModuleWidget.__init__(self, parent)
        VTKObservationMixin.__init__(self)
        self.logic = None
        self.inputVolumeSelector = None
        self.markupNode = None

    def setup(self) -> None:
        ScriptedLoadableModuleWidget.setup(self)

        # One landmark set per analysis type. The active set is always exposed as
        # self.fiducialLabels, so the rest of the module keeps working unchanged.
        anatomicalLabels = {
            "P1": "Diaphysis proximal medial - inner border of cortex",
            "P2": "Diaphysis proximal lateral - inner border of cortex",
            "P3": "Diaphysis distal medial - inner border of cortex",
            "P4": "Diaphysis distal lateral - inner border of cortex",
            "P5": "Diaphysis mid medial - inner cortex",
            "P6": "Diaphysis mid lateral - inner cortex",
            "P7": "Latero-superior greater tuberosity (GT)",
            "P8": "Upper head greater tuberosity junction",
            "P9": "Lower head calcar junction",
            "P10": "Mid head surface",
            "P11": "Lateral fossa - coracoid junction",
            "P12": "Most medial fossa",
            "P13": "Mid fossa",
            "P14": "Lower glenoid",
            "P15": "Upper glenoid",
            "P16": "Lateroinferior acromion",
            "P17": "Medio-inferior acromion",
            "P18": "Scapular neck cortex - Guideline junction",
        }
        # Reverse prosthesis (RSA): same landmarks except P8-P10 (glenosphere),
        # P14 (reamed glenoid) and the two peg points P19-P20.
        reverseLabels = dict(anatomicalLabels)
        reverseLabels.update({
            "P8": "Upper glenosphere",
            "P9": "Lower glenosphere",
            "P10": "Mid glenosphere",
            "P18": "Scapular neck cortex - on the guide line (perpendicular to the fossa line through P8)",
            "P14": "Lower reamed glenoid",
            "P19": "Peg start - limit between glenoid sphere and peg (inferior edge)",
            "P20": "Peg end (inferior edge)",
            "P21": "Upper edge of the humeral component",
            "P22": "Lower edge of the humeral component"
        })
        self.fiducialLabelsByMode = {
            "Anatomical": anatomicalLabels,
            "Reverse prosthesis": reverseLabels,
        }
        self._activeMode = "Anatomical"  # default
        self.fiducialLabels = self.fiducialLabelsByMode[self._activeMode]

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

        # Analysis type: Anatomical (default) or Reverse prosthesis.
        # The two types use different landmarks and different computations.
        self.xrayTypeSelector = qt.QComboBox()
        self.xrayTypeSelector.addItem("Anatomical")
        self.xrayTypeSelector.addItem("Reverse prosthesis")
        self.xrayTypeSelector.setCurrentIndex(0)
        self.xrayTypeSelector.setToolTip(
            "Anatomical: 18 landmarks. Reverse prosthesis: 20 landmarks "
            "(glenosphere, reamed glenoid and peg points)."
        )
        self.selectionFormLayout.addRow("Analysis type:", self.xrayTypeSelector)
        # connect AFTER addItem so it does not fire during setup
        self.xrayTypeSelector.currentIndexChanged.connect(self.onXrayTypeChanged)

        # Patient ID input
        self.patientIdLineEdit = qt.QLineEdit()
        self.patientIdLineEdit.setPlaceholderText("Enter patient ID")
        formLayout.addRow("Patient ID:", self.patientIdLineEdit)

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

        self.populateFiducialTable()

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

        # self.beta_angleExportButton = qt.QPushButton("Export fiducials nodes and parameters")
        # self.beta_angleExportButton.connect('clicked(bool)', lambda: self.onExportFiducials("beta_angle"))
        # self.beta_angleLayout.addWidget(self.beta_angleExportButton)

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

    # ---------- Analysis type (Anatomical / Reverse prosthesis) ----------
    def currentXrayType(self):
        """Return 'Anatomical' or 'Reverse prosthesis'."""
        if getattr(self, "xrayTypeSelector", None) is not None:
            return self.xrayTypeSelector.currentText
        return "Anatomical"

    def populateFiducialTable(self):
        """(Re)fill the guide table with the landmarks of the active analysis type."""
        self.fiducialTable.setRowCount(len(self.fiducialLabels))
        for i, (fidID, label) in enumerate(self.fiducialLabels.items()):
            self.fiducialTable.setItem(i, 0, qt.QTableWidgetItem(fidID))
            self.fiducialTable.setItem(i, 1, qt.QTableWidgetItem(label))

    def clearFiducialsAndOverlays(self):
        """Remove the placed fiducials and every drawn line / circle / arc."""
        scene = slicer.mrmlScene
        for node in list(scene.GetNodesByClass("vtkMRMLMarkupsFiducialNode")):
            if "beta_angle" in (node.GetName() or "").lower():
                scene.RemoveNode(node)
        for node in list(scene.GetNodesByClass("vtkMRMLMarkupsLineNode")):
            scene.RemoveNode(node)
        for node in list(scene.GetNodesByClass("vtkMRMLMarkupsCurveNode")):
            scene.RemoveNode(node)
        self.markupNode = None

    def onXrayTypeChanged(self, index):
        newMode = self.xrayTypeSelector.currentText
        if newMode == self._activeMode or not hasattr(self, "fiducialTable"):
            return

        node = slicer.mrmlScene.GetFirstNodeByName("Fiducials_beta_angle")
        hasWork = node is not None and node.GetNumberOfControlPoints() > 0
        if hasWork and not slicer.util.confirmYesNoDisplay(
                "The two analysis types use different landmarks.\n"
                "Changing it erases the fiducials placed so far and all drawn lines. Continue?"):
            # revert the selector without triggering this handler again
            self.xrayTypeSelector.blockSignals(True)
            self.xrayTypeSelector.setCurrentIndex(self.xrayTypeSelector.findText(self._activeMode))
            self.xrayTypeSelector.blockSignals(False)
            return

        self.clearFiducialsAndOverlays()
        self._activeMode = newMode
        self.fiducialLabels = self.fiducialLabelsByMode[newMode]
        self.populateFiducialTable()
        slicer.util.showStatusMessage(f"Analysis type: {newMode}", 3000)

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
        """Clear any previous fiducials/overlays for this timepoint, then
                activate placement of a fresh set."""
        nodeName = f"Fiducials_{timepoint}"
        scene = slicer.mrmlScene

        # Remove the existing fiducial node for this timepoint, if any
        existingNode = scene.GetFirstNodeByName(nodeName)
        if existingNode:
            scene.RemoveNode(existingNode)

        # Remove all drawn overlays (axis/angle lines, circles, arcs)
        for node in list(scene.GetNodesByClass("vtkMRMLMarkupsLineNode")):
            scene.RemoveNode(node)
        for node in list(scene.GetNodesByClass("vtkMRMLMarkupsCurveNode")):
            scene.RemoveNode(node)

        # Create a fresh fiducial node
        self.markupNode = scene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode", nodeName)
        self.markupNode.AddObserver(slicer.vtkMRMLMarkupsNode.PointAddedEvent, self.onFiducialAdded)

        if "beta_angle" in timepoint.lower():
            displayNode = self.markupNode.GetDisplayNode()
            if displayNode:
                displayNode.SetSelectedColor(0, 1, 0)  # green
            # displayNode.SetUseGlyphScale(False)
            # displayNode.SetGlyphSize(1.5)

        # Start place mode (1 point at a time)
        slicer.modules.markups.logic().StartPlaceMode(1)

    def onFiducialAdded(self, caller, event):
        """Automatically label new fiducials P1..PN up to the number of labels provided."""
        n = caller.GetNumberOfControlPoints()
        if n <= len(self.fiducialLabels):
            fidName = f"P{n}"
            caller.SetNthControlPointLabel(n - 1, fidName)

            # Guide line for the scapular neck point P18: drawn right after P17 is placed.
            if n == 18:
                self.drawGuideLineForNeckPoint(caller)
        else:
            # Stop placement if too many points
            interactionNode = slicer.app.applicationLogic().GetInteractionNode()
            interactionNode.SetCurrentInteractionMode(slicer.vtkMRMLInteractionNode.ViewTransform)
            slicer.util.showStatusMessage("Maximum fiducials placed — placement stopped.", 3000)

    def activeVolumeNode(self):
        """Volume used for the analysis: the selected input volume, else the first scalar volume."""
        node = None
        try:
            node = self.inputVolumeSelector.currentNode() if self.inputVolumeSelector else None
        except Exception:
            node = None
        if node is None:
            node = slicer.mrmlScene.GetFirstNodeByClass("vtkMRMLScalarVolumeNode")
        return node

    def imageFrame(self, points):
        """In-plane frame of the 2D radiograph. points: {label: {x, y, z}}.
        1) image in the x-y, x-z or y-z plane -> the two varying coordinates (constant one ignored)
        2) otherwise (oblique image) -> best-fit plane of the placed points."""
        pts3 = [[p['x'], p['y'], p['z']] for p in points.values()]
        frame = self.logic.axis_plane_frame(pts3)
        if frame is not None:
            logging.info("Image plane detected: %s", frame[3])
            return frame
        logging.info("Oblique image: using the best-fit plane of the points")
        return self.logic.plane_frame(pts3)

    def drawGuideLineForNeckPoint(self, markupNode):
        """
        Temporary guide line to help place P18 (scapular neck), drawn after P17.

        Reverse prosthesis: perpendicular to the fossa line (TLS fit P11-P13) through
        P8 (upper glenosphere); P18 goes on the neck cortex along this line.
        Anatomical: line through P14 parallel to the fossa line (TLS fit P11-P13).
        The line spans the whole image (half-length = image diagonal).
        """
        try:
            reverse = (self.currentXrayType() == "Reverse prosthesis")
            points = self.logic.get_fiducial_positions(markupNode)
            required = ("P11", "P12", "P13", "P8" if reverse else "P14")
            missing = [p for p in required if p not in points]
            if missing:
                logging.warning("Neck guide line skipped: missing %s", missing)
                return

            frame = self.imageFrame(points)
            pt = lambda k: self.logic.to_plane([points[k]['x'], points[k]['y'], points[k]['z']], frame)
            _, fossa_dir = self.logic.fit_line_tls([pt("P11"), pt("P12"), pt("P13")])

            if reverse:
                direction = np.array([-fossa_dir[1], fossa_dir[0], 0.0])
                anchor = pt("P8")
            else:
                direction = np.array([-fossa_dir[1], fossa_dir[0], 0.0])
                anchor = pt("P15")

            # Long enough to cross the whole image whatever the pixel spacing
            half_length = 250.0
            volumeNode = self.activeVolumeNode()
            if volumeNode is not None:
                bounds = [0.0] * 6
                volumeNode.GetBounds(bounds)
                diagonal = float(np.linalg.norm([bounds[1] - bounds[0],
                                                 bounds[3] - bounds[2],
                                                 bounds[5] - bounds[4]]))
                if diagonal > 0:
                    half_length = diagonal

            self.logic.draw_line(self.logic.from_plane(anchor - half_length * direction, frame),
                                 self.logic.from_plane(anchor + half_length * direction, frame),
                                 color=(0, 1, 0), name="NeckGuideLine")

            # Creating the guide node hijacks the active placement target:
            # point it back at the fiducial node and re-arm placement.
            selectionNode = slicer.app.applicationLogic().GetSelectionNode()
            selectionNode.SetActivePlaceNodeClassName(markupNode.GetClassName())
            selectionNode.SetActivePlaceNodeID(markupNode.GetID())
            slicer.modules.markups.logic().StartPlaceMode(1)

        except Exception as e:
            logging.exception("drawGuideLineForNeckPoint failed")
            slicer.util.showStatusMessage(f"Neck guide line failed: {e}", 8000)

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

        outputDir = self.getResultsDirectory()

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
            pdfName = "Guide_Placement.pdf"
            if self.currentXrayType() == "Reverse prosthesis":
                # optional dedicated guide; falls back to the standard one if absent
                if os.path.exists(os.path.join(moduleDir, "Resources", "Guide_Placement_Reverse.pdf")):
                    pdfName = "Guide_Placement_Reverse.pdf"
            pdfPath = os.path.join(moduleDir, "Resources", pdfName)

            if not os.path.exists(pdfPath):
                slicer.util.errorDisplay("PDF file not found.")
                return

            qt.QDesktopServices.openUrl(qt.QUrl.fromLocalFile(pdfPath))

        except Exception as e:
            slicer.util.errorDisplay(f"Failed to open PDF: {str(e)}")

    def onCalculateMetrics(self, timepoint: str):
        """Compute the metrics of the selected analysis type, show them and export.

        Anatomical (18 landmarks)        -> computeAnatomicalMetrics
        Reverse prosthesis (20 landmarks) -> computeReverseMetrics
        """
        nodeName = f"Fiducials_{timepoint}"
        scene = slicer.mrmlScene
        markupNode = scene.GetFirstNodeByName(nodeName)

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
        #  for label in points:
        #      points[label]['z'] = 0

        # Remove all drawn overlays (axis/angle lines, circles, arcs)
        for node in list(scene.GetNodesByClass("vtkMRMLMarkupsLineNode")):
            scene.RemoveNode(node)
        for node in list(scene.GetNodesByClass("vtkMRMLMarkupsCurveNode")):
            scene.RemoveNode(node)

        if self.currentXrayType() == "Reverse prosthesis":
            metrics, _ = self.computeReverseMetrics(points, patientID)
        else:
            metrics, _ = self.computeAnatomicalMetrics(points, patientID)
        if metrics is None:
            return

        # Convert the distances to real mm (DICOM first) and let the user confirm the scale
        metrics = self.calibrateMetrics(metrics)
        if metrics is None:
            slicer.util.showStatusMessage("Calculation cancelled.", 3000)
            return
        text = self.buildResultsText(metrics, patientID)

        self.currentMetrics = metrics
        self.showTextInSliceViews(text)
        self.exportAfterCompute(markupNode, metrics, patientID, timepoint)

    def computeAnatomicalMetrics(self, points, patientID):
        """Pre-operative (anatomical) analysis, 18 landmarks. Returns (metrics, text)."""
        return self._computeShoulderMetrics(points, patientID, "Anatomical", 18)

    def computeReverseMetrics(self, points, patientID):
        """Reverse prosthesis (RSA) analysis, 20 landmarks. Returns (metrics, text)."""
        return self._computeShoulderMetrics(points, patientID, "Reverse prosthesis", 20)

    def _computeShoulderMetrics(self, points, patientID, mode, nPoints):
        try:
            P3 = {f"P{i}": np.array([points[f"P{i}"]['x'], points[f"P{i}"]['y'], points[f"P{i}"]['z']])
                  for i in range(1, nPoints + 1)}
            frame = self.imageFrame(points)
            P = {k: self.logic.to_plane(v, frame) for k, v in P3.items()}
            m = self.logic.compute_shoulder_metrics(P, mode, frame=frame)
        except Exception as e:
            slicer.util.errorDisplay(f"{mode} analysis failed: {e}\n"
                                     "Check the placement of the points.")
            return None, None

        return m, ""

    # ---------- Scale (calibration) ----------
    def readDicomTags(self, volumeNode):
        """Spacing-related DICOM tags of the loaded image: {'PixelSpacing': '0.14\\0.14', ...}."""
        tags = {}
        wanted = (("PixelSpacing", "0028,0030"), ("ImagerPixelSpacing", "0018,1164"),
                  ("Magnification", "0018,1114"), ("CalibrationType", "0028,0A02"))
        try:
            uids = (volumeNode.GetAttribute("DICOM.instanceUIDs") or "").split()
            if not uids:
                return tags
            path = slicer.dicomDatabase.fileForInstance(uids[0])
            if not path:
                return tags
            for key, tag in wanted:
                try:
                    value = slicer.dicomDatabase.fileValue(path, tag)
                except Exception:
                    value = ""
                if value:
                    tags[key] = str(value)
            if not tags:  # fall back on pydicom when available
                try:
                    import pydicom
                    ds = pydicom.dcmread(path, stop_before_pixels=True)
                    for key, kw in (("PixelSpacing", "PixelSpacing"),
                                    ("ImagerPixelSpacing", "ImagerPixelSpacing"),
                                    ("Magnification", "EstimatedRadiographicMagnificationFactor"),
                                    ("CalibrationType", "PixelSpacingCalibrationType")):
                        if kw in ds:
                            tags[key] = "\\".join(str(v) for v in ds[kw].value) \
                                if hasattr(ds[kw].value, "__iter__") and not isinstance(ds[kw].value, str) \
                                else str(ds[kw].value)
                except Exception:
                    pass
        except Exception:
            logging.warning("Could not read the DICOM spacing tags", exc_info=True)
        return tags

    def calibrateMetrics(self, m):
        """
        Put the distances in real mm and ask the user whether the scale looks right.
        Priority: DICOM spacing (PixelSpacing, else ImagerPixelSpacing / magnification),
        then the spacing Slicer assigned to the image. Returns the metrics, or None if
        the user cancelled.
        """
        mode = m["xray_type"]
        factor, source = 1.0, "image spacing as loaded in Slicer (no DICOM spacing found)"
        volumeNode = self.activeVolumeNode()
        volSpacing = None
        if volumeNode is not None:
            try:
                sx, sy, _ = volumeNode.GetSpacing()
                volSpacing = (float(sx), float(sy))
                m["volume_spacing_mm"] = volSpacing
            except Exception:
                volSpacing = None
            dicomSpacing, dicomSource = self.logic.choose_dicom_spacing(self.readDicomTags(volumeNode))
            if dicomSpacing and volSpacing:
                factor = self.logic.scale_factor(dicomSpacing, volSpacing)
                source = dicomSource
                m["dicom_spacing_mm"] = dicomSpacing
        else:
            source = "no image found"

        m = self.logic.apply_scale(m, factor)
        check = self.logic.scale_plausibility(m, mode)

        volumeId = volumeNode.GetID() if volumeNode is not None else None
        confirmed = getattr(self, "_confirmedScale", {})
        key = (volumeId, round(factor, 4))
        userConfirmed = False
        if not (check["ok"] and key in confirmed):
            vs = (f"{volSpacing[0]:.3f} x {volSpacing[1]:.3f} mm" if volSpacing else "unknown")
            ds = m.get("dicom_spacing_mm")
            dsText = f"{ds[0]:.3f} x {ds[1]:.3f} mm" if ds else "not available"
            text = (f"Scale check\n\n"
                    f"Spacing source: {source}\n"
                    f"DICOM spacing: {dsText}\n"
                    f"Spacing in Slicer: {vs}\n"
                    f"Correction applied: x{factor:.3f}\n\n"
                    f"Fitted circle diameter: {check['diameter']:.1f} mm\n"
                    f"Expected {check['label']}: {check['low']:.0f}-{check['high']:.0f} mm\n")
            if not check["ok"]:
                text += "\nWARNING: the diameter is outside the expected range - the scale may be wrong.\n"
            text += "\nIs this scale plausible?"
            if slicer.util.confirmYesNoDisplay(text, windowTitle="Check the scale"):
                userConfirmed = True
            else:
                dlg = qt.QInputDialog(slicer.util.mainWindow())
                dlg.setWindowTitle("Manual calibration")
                dlg.setLabelText(f"Real diameter (mm) of the fitted circle\n({check['label']}):")
                dlg.setInputMode(qt.QInputDialog.DoubleInput)
                dlg.setDoubleDecimals(1)
                dlg.setDoubleRange(1.0, 300.0)
                dlg.setDoubleValue(float(check["typical"]))
                if dlg.exec_() != qt.QDialog.Accepted:
                    return None
                manual = float(dlg.doubleValue) / check["diameter"]
                m = self.logic.apply_scale(m, manual)
                factor *= manual
                source = f"manual calibration (circle diameter set to {dlg.doubleValue:.1f} mm)"
                userConfirmed = True
        if userConfirmed:
            confirmed[(volumeId, round(factor, 4))] = True
            self._confirmedScale = confirmed

        m["scale_factor_applied"] = float(factor)
        m["scale_source"] = source
        m["scale_confirmed_by_user"] = bool(userConfirmed or (check["ok"] and key in confirmed))
        return m

    def buildResultsText(self, m, patientID):
        mode = m["xray_type"]
        lines = [f"Patient: {patientID}  ({'RSA' if mode == 'Reverse prosthesis' else 'pre-op'})\n",
                 "-------------------------\n"]
        if "scale_factor_applied" in m:
            lines.append(f"Scale: x{m['scale_factor_applied']:.3f}\n")
        if "COR_radius" in m:
            lines.append(f"Circle diameter: {2 * m['COR_radius']:.1f} mm\n")
        if mode == "Reverse prosthesis":
            lines += [f"RSAA : {_fmt(m['RSAA'], '°')}\n",
                      f"DistA: {_fmt(m['DistA'], '°')}\n",
                      f"LatA : {_fmt(m['LatA'], '°')}\n"]
        else:
            lines += [f"GIA  : {_fmt(m['GIA'], '°')}\n",
                      f"CSA  : {_fmt(m['CSA'], '°')}\n"]
        lines += [f"CORLat   : {_fmt(m['CORLat'], ' mm')}\n",
                  f"CORDist  : {_fmt(m['CORDist'], ' mm')}\n",
                  f"HumOffset: {_fmt(m['HumOffset'], ' mm')}\n",
                  f"GTHeight : {_fmt(m['GTHeight'], ' mm')}\n",
                  f"GTLat    : {_fmt(m['GTLat'], ' mm')}\n",
                  f"NeckLat  : {_fmt(m['NeckLat'], ' mm')}\n",
                  f"NeckDist : {_fmt(m['NeckDist'], ' mm')}\n"]
        return "".join(lines)

    def showTextInSliceViews(self, text):
        """Display the results in the corner of the slice views."""
        layoutManager = slicer.app.layoutManager()

        for viewName in ["Red", "Yellow", "Green"]:
            sliceWidget = layoutManager.sliceWidget(viewName)
            if sliceWidget:
                sliceView = sliceWidget.sliceView()
                cornerAnnotation = sliceView.cornerAnnotation()
                cornerAnnotation.SetText(vtk.vtkCornerAnnotation.UpperLeft, text)
                sliceView.forceRender()

    def getResultsDirectory(self):
        """
        Folder 'Results' inside the module (created if needed).
        """
        resultsDir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Results")
        try:
            os.makedirs(resultsDir, exist_ok=True)
            if not os.access(resultsDir, os.W_OK):
                raise PermissionError(resultsDir)
        except Exception:
            fallback = os.path.join(slicer.app.temporaryPath, "Beta_angle_ana_Results")
            os.makedirs(fallback, exist_ok=True)
            logging.warning("Results folder %s not writable, using %s", resultsDir, fallback)
            slicer.util.warningDisplay(
                f"The module's Results folder is not writable.\nSaving to:\n{fallback}")
            return fallback
        return resultsDir

    def exportAfterCompute(self, markupNode, metrics, patientID, timepoint):
        """Save fiducials + metrics right after computing, in the module's Results folder."""
        outputDir = self.getResultsDirectory()
        try:
            self.logic.saveCombinedCSV(markupNode, metrics, patientID, timepoint, outputDir)
            slicer.util.showStatusMessage(f"Fiducials + metrics saved in: {outputDir}", 6000)
        except Exception as e:
            slicer.util.errorDisplay(f"Export failed: {e}")

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

        modeTag = {"Anatomical": "preop", "Reverse prosthesis": "RSA"}.get(
            metrics.get("xray_type", ""), "")
        fileName = f"{patientID}_{modeTag + '_' if modeTag else ''}{timepoint}_combined.csv"
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
        """
        axis_vec = axisB - axisA
        axis_len = np.linalg.norm(axis_vec)
        if axis_len == 0:
            return 0.0
        axis_unit = axis_vec / axis_len
        return float(np.dot(vec, axis_unit))

    # --- Order two points as (upper, lower) using a "superior" direction hint ---
    def order_upper_lower(self, pA, pB, up_hint):
        """
        Return (upper, lower) for two points, so the result does not depend on
        the order in which the user clicked them. `up_hint` is any vector that
        points roughly toward the patient's superior side in the image.
        """
        if np.dot((pA - pB)[:2], np.asarray(up_hint)[:2]) >= 0:
            return pA, pB
        return pB, pA

    # --- frame ---
    @staticmethod
    def plane_frame(points3d):
        pts = np.asarray(points3d, dtype=float)
        origin = pts.mean(axis=0)
        _, sv, vt = np.linalg.svd(pts - origin, full_matrices=False)
        if len(sv) > 2 and sv[1] > 0 and sv[2] > 0.05 * sv[1]:
            logging.warning("Fiducials are not coplanar (singular values %s)", sv)
        return origin, vt[0], vt[1]

    @staticmethod
    def axis_plane_frame(points3d, tol=0.01):
        """
        2D radiograph lying in the x-y, x-z or y-z plane: keep the two coordinates that vary
        (in their natural order) and ignore the constant one. Returns (origin, e1, e2, name),
        or None when the points are not in an axis-aligned plane (oblique image).
        """
        pts = np.asarray(points3d, dtype=float)
        spread = np.ptp(pts, axis=0)
        flat = int(np.argmin(spread))  # the constant axis
        if spread.max() <= 0 or spread[flat] > tol * spread.max():
            return None
        keep = [a for a in range(3) if a != flat]
        e1, e2 = np.eye(3)[keep[0]], np.eye(3)[keep[1]]
        origin = np.zeros(3)
        origin[flat] = float(pts[:, flat].mean())
        return origin, e1, e2, "xyz"[keep[0]] + "-" + "xyz"[keep[1]]

    @staticmethod
    def to_plane(p, frame):
        origin, e1, e2 = frame[:3]
        v = np.asarray(p, dtype=float) - origin
        return np.array([float(v @ e1), float(v @ e2), 0.0])

    @staticmethod
    def from_plane(q, frame):
        origin, e1, e2 = frame[:3]
        q = np.asarray(q, dtype=float)
        return origin + q[0] * e1 + q[1] * e2

    # --- Scale helpers ---
    MM_KEYS = ("CORLat", "CORDist", "HumOffset", "GTHeight", "GTLat", "NeckLat", "NeckDist",
               "COR_radius")

    @staticmethod
    def parse_dicom_spacing(value):
        """'0.14\\0.15' / '0.14,0.15' / [0.14, 0.15] -> (row, col) floats, or None."""
        if value is None:
            return None
        try:
            if isinstance(value, str):
                parts = value.replace("\\", " ").replace(",", " ").replace(";", " ").split()
            else:
                parts = list(value)
            vals = [float(p) for p in parts]
            if len(vals) == 1:
                vals = [vals[0], vals[0]]
            if len(vals) < 2 or min(vals[:2]) <= 0:
                return None
            return (vals[0], vals[1])
        except Exception:
            return None

    def choose_dicom_spacing(self, tags):
        """
        Pixel spacing in the patient plane from DICOM tags, by priority:
          1. PixelSpacing (0028,0030)
          2. ImagerPixelSpacing (0018,1164) / EstimatedRadiographicMagnificationFactor (0018,1114)
          3. ImagerPixelSpacing alone (detector plane: magnification NOT corrected)
        Returns ((row, col), source) or (None, "").
        """
        tags = tags or {}
        ps = self.parse_dicom_spacing(tags.get("PixelSpacing"))
        ips = self.parse_dicom_spacing(tags.get("ImagerPixelSpacing"))
        try:
            mag = float(str(tags.get("Magnification", "")).replace("\\", " ").split()[0])
        except Exception:
            mag = None
        if ps:
            calib = str(tags.get("CalibrationType", "")).strip()
            return ps, "DICOM PixelSpacing (0028,0030)" + (f", calibration type {calib}" if calib else "")
        if ips and mag and mag > 0:
            return (ips[0] / mag, ips[1] / mag), f"DICOM ImagerPixelSpacing / magnification {mag:.3f}"
        if ips:
            return ips, "DICOM ImagerPixelSpacing (detector plane, magnification not corrected)"
        return None, ""

    @staticmethod
    def scale_factor(dicom_spacing, volume_spacing_xy):
        """Factor turning distances measured with Slicer's spacing into distances with the DICOM
        spacing. dicom_spacing = (row, col); volume_spacing_xy = (sx, sy) with sx along columns."""
        sx, sy = volume_spacing_xy
        f = 0.5 * (dicom_spacing[0] / sy + dicom_spacing[1] / sx)
        return 1.0 if abs(f - 1.0) < 0.005 else float(f)

    def apply_scale(self, metrics, factor):
        """Multiply every distance (mm) by `factor`; angles are unchanged."""
        if factor == 1.0:
            return dict(metrics)
        out = dict(metrics)
        for k in self.MM_KEYS:
            if k in out and out[k] is not None:
                out[k] = out[k] * factor
        if "HumeralAxis" in out:
            out["HumeralAxis"] = np.asarray(out["HumeralAxis"]) * factor
        return out

    @staticmethod
    def scale_plausibility(metrics, mode):
        """Is the fitted circle (humeral head / glenosphere) a plausible size in mm?"""
        diameter = 2.0 * float(metrics["COR_radius"])
        if mode == "Reverse prosthesis":
            low, high, typical, label = 30.0, 48.0, 39.0, "glenosphere diameter (nominal 36, 39 or 42 mm)"
        else:
            low, high, typical, label = 35.0, 60.0, 46.0, "humeral head diameter (typically 40-55 mm)"
        return {"diameter": diameter, "low": low, "high": high, "typical": typical,
                "label": label, "ok": low <= diameter <= high}

    # --- Metrics of both analyses (pre-op / reverse prosthesis) ---
    def compute_shoulder_metrics(self, P, mode, frame=None):
        """
        P    : dict {"P1": np.array([x, y, 0]), ...}  (18 points pre-op, 20 points RSA)
        mode : "Anatomical" (pre-op) or "Reverse prosthesis" (RSA)

        Landmarks (same in both analyses unless stated):
          P1-P6   diaphysis (inner cortex: medial P1/P3/P5, lateral P2/P4/P6)
          P7      latero-superior greater tuberosity
          P8-P10  humeral head (pre-op)  /  glenosphere upper, lower, mid (RSA)
          P11-P13 fossa (lateral, most medial, mid)
          P14     lower glenoid (pre-op) / lower reamed glenoid (RSA)
          P15     upper glenoid
          P16,P17 acromion
          P18     scapular neck
          P19,P20 peg start / end (RSA only)
          P21, P22 humeral component (RSA only)

        Definitions (mm, degrees):
          Fossa line     TLS fit through P11-P13 (the P11-P12 line)
          lateral dir    fossa direction pointing toward the humerus
          distal dir     perpendicular to the fossa line, pointing toward the lower glenoid
          COR            centre of the circle through P8-P10 (humeral head / glenosphere)
          SRP            foot of the perpendicular from the reference point onto the fossa line
                         (reference point = P8 upper glenosphere in RSA, P15 upper glenoid pre-op;
                         in RSA, P18 is placed on that same perpendicular)
          Pre-op only    GIA  90 - Beta, Beta = 180 - angle(fossa line (lateral), glenoid line P14->P15);
                              + superior inclination, - inferior inclination
                         CSA  angle at P14 between P15 and P16
          RSA only       RSAA glenosphere line P8-P9 vs perpendicular to the fossa line, signed:
                              + superior inclination, - inferior inclination
                         DistA angle at P15 between P16 and P7
                         LatA  angle at P16 between P15 and P7
          Both           CORLat/CORDist  COR relative to SRP, along the lateral / distal dir (signed)
                         HumOffset       distance COR - humeral axis
                         GTHeight        P7 relative to COR along the humeral axis (+ = proximal)
                         GTLat           distance P7 - humeral axis
                         NeckLat/NeckDist  P14 -> P18 along the lateral / distal dir (absolute)
        """
        is_rsa = (mode == "Reverse prosthesis")
        P = {k: np.array([v[0], v[1], 0.0], dtype=float) for k, v in P.items()}
        if frame is not None:
            to3d = lambda q: self.from_plane(q, frame)
            circle_axes = (frame[1], frame[2])
        else:
            to3d = lambda q: np.asarray(q, dtype=float)
            circle_axes = None
        draw_line = lambda A, B, **kw: self.draw_line(to3d(A), to3d(B), **kw)
        draw_circle = lambda c, r, **kw: self.draw_circle(to3d(c), r, axes=circle_axes, **kw)

        # --- Humeral axis (proximal direction positive) ---
        res = self.central_axis_from_cortices(
            [P["P1"], P["P3"], P["P5"]], [P["P2"], P["P4"], P["P6"]]
        )
        axis_origin = res["origin"]
        axis_dir = res["direction"]
        prox = 0.5 * (P["P1"] + P["P2"]) - 0.5 * (P["P3"] + P["P4"])
        if np.dot(axis_dir, prox) < 0:
            axis_dir = -axis_dir
        axis_vector = res["p_end"] - res["p_start"]
        draw_line(res["p_start"], res["p_end"], color=(1, 1, 0), name="HumeralAxis")

        # --- Fossa line and its two directions ---
        fossa_centroid, fossa_dir = self.fit_line_tls([P["P11"], P["P12"], P["P13"]])
        humerus_centroid = np.mean([P[f"P{i}"] for i in range(1, 7)], axis=0)
        lat_dir = fossa_dir if np.dot(fossa_dir, humerus_centroid - fossa_centroid) >= 0 else -fossa_dir
        dist_dir = np.array([-lat_dir[1], lat_dir[0], 0.0])
        if np.dot(dist_dir, P["P14"] - P["P15"]) < 0:
            dist_dir = -dist_dir

        # --- Centre of rotation: circle through P8, P9, P10 ---
        cor, radius = self.compute_circle_center(P["P8"], P["P9"], P["P10"])
        if cor is None:
            raise ValueError(
                "P8, P9 and P10 are (almost) on the same line - cannot fit the circle. "
                f"P8=({P['P8'][0]:.2f}, {P['P8'][1]:.2f}) "
                f"P9=({P['P9'][0]:.2f}, {P['P9'][1]:.2f}) "
                f"P10=({P['P10'][0]:.2f}, {P['P10'][1]:.2f})")
        cor = np.array([cor[0], cor[1], 0.0])
        draw_circle(cor, radius, name="GlenosphereCircle" if is_rsa else "HumeralHeadCircle")

        # --- Scapular reference point (SRP) ---
        ref = P["P8"] if is_rsa else P["P15"]
        srp = fossa_centroid + np.dot(ref - fossa_centroid, fossa_dir) * fossa_dir

        # --- Drawings ---
        fossa_pts = np.vstack([P["P11"], P["P12"], P["P13"], srp])
        t = (fossa_pts - fossa_centroid) @ fossa_dir
        draw_line(fossa_centroid + t.min() * fossa_dir, fossa_centroid + t.max() * fossa_dir,
                  color=(0, 1, 1), name="FossaAxis")
        draw_line(ref, srp, color=(0.7, 0.7, 0.7), name="SRP_perpendicular")
        draw_line(srp, cor, color=(1, 0.5, 0), name="SRP_COR")

        # --- Measures common to both analyses ---
        def perp_dist(pt):  # unsigned distance to the humeral axis
            v = pt - axis_origin
            return float(abs(v[0] * axis_dir[1] - v[1] * axis_dir[0]))

        d_cor = cor - srp
        neck = P["P18"] - P["P14"]
        common = {
            "CORLat": float(np.dot(d_cor, lat_dir)),
            "CORDist": float(np.dot(d_cor, dist_dir)),
            "HumOffset": perp_dist(cor),
            "GTHeight": float(np.dot(P["P7"] - cor, axis_dir)),
            "GTLat": perp_dist(P["P7"]),
            "NeckLat": float(abs(np.dot(neck, lat_dir))),
            "NeckDist": float(abs(np.dot(neck, dist_dir))),
        }

        if is_rsa:
            # RSAA: tilt of the glenosphere line from the perpendicular to the fossa line.
            # g points distally; its lateral lean gives the sign (+ superior inclination).
            g = P["P8"] - P["P9"]
            if np.dot(g, dist_dir) < 0:
                g = -g
            g = g / np.linalg.norm(g)
            rsaa = float(np.degrees(np.arctan2(abs(np.dot(g, lat_dir)), np.dot(g, dist_dir))))
            if np.dot(g, lat_dir) < 0:
                rsaa = -rsaa
            draw_line(P["P8"], P["P9"], color=(0.5, 0, 1), name="GlenosphereLine")
            draw_line(P["P19"], P["P20"], color=(1, 0.5, 0), name="PegAxis")
            specific = {
                "RSAA": rsaa,
                "DistA": self.compute_angle(P["P16"], P["P15"], P["P7"]),
                "LatA": self.compute_angle(P["P15"], P["P16"], P["P7"]),
            }
        else:
            draw_line(P["P14"], P["P15"], color=(0.5, 0, 1), name="GlenoidLine")
            # Beta: 180 - angle(fossa line pointing laterally, glenoid line pointing superiorly)
            # GIA = 90 - Beta (+ superior inclination, - inferior inclination)
            beta = self.compute_beta_angle_from_vectors(lat_dir, P["P15"] - P["P14"])
            gia = float(90.0 - beta)
            specific = {
                "GIA": gia,
                "CSA": self.compute_angle(P["P15"], P["P14"], P["P16"]),
            }

        out = {"xray_type": mode}
        out.update(specific)
        out.update(common)
        out["COR"] = cor
        out["COR_radius"] = float(radius)
        out["SRP"] = srp
        out["HumeralAxis"] = axis_vector
        return out

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
        return dist  # , closest

    # Find intersection axis-circle
    def find_axis_circle_intersection(self, origin, direction, center, radius, reference_point):
        """
        Find where the line (origin + t*direction) crosses the circle
        (center, radius), in the image plane.
        """
        O2 = np.asarray(origin[:2], dtype=float)
        d2 = np.asarray(direction[:2], dtype=float)
        d2 = d2 / np.linalg.norm(d2)
        C2 = np.asarray(center[:2], dtype=float)

        V = O2 - C2
        b = 2 * np.dot(V, d2)
        c = np.dot(V, V) - radius ** 2
        disc = b ** 2 - 4 * c

        if disc < 0:
            return None

        sqrt_disc = np.sqrt(disc)
        t1 = (-b + sqrt_disc) / 2
        t2 = (-b - sqrt_disc) / 2

        X1 = O2 + t1 * d2
        X2 = O2 + t2 * d2

        ref2 = np.asarray(reference_point[:2], dtype=float)
        chosen_xy = X1 if np.linalg.norm(X1 - ref2) <= np.linalg.norm(X2 - ref2) else X2

        z = origin[2] if len(origin) > 2 else 0.0
        return np.array([chosen_xy[0], chosen_xy[1], z])

    # --- Vertical distance from a line segment down to a circle ---
    def compute_vertical_distance_to_circle(self, p16, p17, center, radius):
        """
        'Lower' the line P16-P17 straight down until it becomes tangent to the circle, and
        return that vertical drop.

        Returns vertical_distance: signed drop in mm. Positive = circle is
            below the line (normal SAS case). Negative = the line already
            overlaps the circle at that x (should be flagged, not trusted).
        """
        x1, y1 = p16[:2]
        x2, y2 = p17[:2]
        cx, cy, cz = center
        dx, dy = (x2 - x1), (y2 - y1)

        if abs(dx) < 1e-9:
            raise ValueError("P16-P17 is vertical - vertical-translation "
                             "distance is undefined. Check point placement.")

        m = dy / dx
        norm_unit = np.array([-m, 1.0]) / np.sqrt(1 + m ** 2)

        tangentA = np.array([cx, cy]) + radius * norm_unit
        tangentB = np.array([cx, cy]) - radius * norm_unit

        def vertical_gap(tangent_xy):
            tx, ty = tangent_xy
            y_on_line = y1 + m * (tx - x1)
            return y_on_line - ty

        gapA, gapB = vertical_gap(tangentA), vertical_gap(tangentB)
        vertical_distance, tangent_xy = (
            (gapA, tangentA) if abs(gapA) <= abs(gapB) else (gapB, tangentB)
        )

        tangent_point = np.array([tangent_xy[0], tangent_xy[1], cz])
        origin_point = np.array([tangent_xy[0], y1 + m * (tangent_xy[0] - x1), cz])
        return float(vertical_distance), origin_point, tangent_point

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
    @staticmethod
    def _hide_measurements(node):
        """Keep only the node name in the label: disable length/angle/etc. measurements."""
        try:
            for i in range(node.GetNumberOfMeasurements()):
                node.GetNthMeasurement(i).SetEnabled(False)
            node.Modified()
        except Exception:
            logging.exception("could not disable measurements")

    def draw_arc(self, vertex, p1, p2, angle_deg, name="Arc", color=(1, 1, 0), thickness=0.8, num_points=30):
        """
        Draw an arc centered at `vertex`, starting at `p1` and spanning `angle_deg` toward `p2`.
        The arc is drawn in the XY plane.
        """
        for node in slicer.mrmlScene.GetNodesByClass("vtkMRMLMarkupsCurveNode"):
            if node.GetName() == name:
                slicer.mrmlScene.RemoveNode(node)

        v_start = p1 - vertex
        angle_rad = np.radians(angle_deg)
        points = []
        for t in np.linspace(0, 1, num_points):
            theta = t * angle_rad
            rot_matrix = np.array([[np.cos(theta), -np.sin(theta)],
                                   [np.sin(theta), np.cos(theta)]])
            arc_point_xy = vertex[:2] + rot_matrix @ v_start[:2]
            points.append([arc_point_xy[0], arc_point_xy[1], vertex[2]])

        curveNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsCurveNode", name)
        for pt in points:
            curveNode.AddControlPoint(pt)
        self._hide_measurements(curveNode)

        displayNode = curveNode.GetDisplayNode()
        if displayNode:
            displayNode.SetSelectedColor(*color)

            # Absolute line thickness (mm), independent of glyph size
            displayNode.SetCurveLineSizeMode(displayNode.UseLineDiameter)
            displayNode.SetLineDiameter(thickness)

            # Hide point markers - safe now the line doesn't depend on glyph size
            displayNode.SetGlyphScale(0.0)
            displayNode.SetVisibility2D(True)
            displayNode.SetVisibility3D(True)
            displayNode.SetPointLabelsVisibility(False)
            displayNode.SetPropertiesLabelVisibility(True)

    # --- Draw line ---
    def draw_line(self, A, B, color=(1, 1, 0), thickness=0.8, name=None):
        """Draw a line as a Markups Line Node, with endpoint markers hidden."""
        # Remove previous line node with same name
        for node in slicer.mrmlScene.GetNodesByClass("vtkMRMLMarkupsLineNode"):
            if node.GetName() == name:
                slicer.mrmlScene.RemoveNode(node)

        lineNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsLineNode", name or "")
        lineNode.AddControlPoint(A.tolist())
        lineNode.AddControlPoint(B.tolist())
        lineNode.SetNthControlPointLabel(0, "")
        lineNode.SetNthControlPointLabel(1, "")
        self._hide_measurements(lineNode)

        displayNode = lineNode.GetDisplayNode()
        if displayNode:
            displayNode.SetSelectedColor(*color)

            # Absolute line thickness (mm), independent of glyph size
            displayNode.SetCurveLineSizeMode(displayNode.UseLineDiameter)
            displayNode.SetLineDiameter(thickness)

            # Hide the two endpoint dot markers entirely
            displayNode.SetGlyphScale(0.0)

            displayNode.SetVisibility2D(True)
            displayNode.SetVisibility3D(True)

            # Show only the line's name (as a single label), never per-point labels
            displayNode.SetPointLabelsVisibility(False)
            displayNode.SetPropertiesLabelVisibility(bool(name))

        return lineNode

    # --- Calculate the center of the circle with 3 reference point to find the center of rotation of the humeral head ---
    def compute_circle_center(self, A, B, C):
        """
        Compute circle center passing through 3 points (2D).
        """
        ax, ay = A[:2]
        bx, by = B[:2]
        cx, cy = C[:2]

        d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))

        # Collinearity test relative to the size of the triangle (d is twice the
        # signed area x 2, so it scales with length^2): independent of units and
        # of which side of the line P8-P10 the third point lies on.
        scale = max(np.hypot(ax - bx, ay - by), np.hypot(bx - cx, by - cy),
                    np.hypot(ax - cx, ay - cy)) ** 2
        if scale == 0 or abs(d) < 1e-9 * scale:
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

    def draw_circle(self, center, radius, name="Circle", color=(0, 1, 1), num_points=80, thickness=0.8, axes=None):
        """Circle in the plane spanned by `axes` = (e1, e2) (default: the x-y plane)."""
        e1, e2 = axes if axes is not None else (np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0]))

        for node in slicer.mrmlScene.GetNodesByClass("vtkMRMLMarkupsCurveNode"):
            if node.GetName() == name:
                slicer.mrmlScene.RemoveNode(node)

        curveNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsCurveNode", name)

        for t in np.linspace(0, 2 * np.pi, num_points):
            curveNode.AddControlPoint((np.asarray(center, float) + radius * (np.cos(t) * e1 + np.sin(t) * e2)).tolist())
        self._hide_measurements(curveNode)

        displayNode = curveNode.GetDisplayNode()
        displayNode.SetSelectedColor(*color)

        # Make line thickness an ABSOLUTE size (mm), independent of glyph size,
        # so hiding the glyphs below doesn't also hide the line.
        displayNode.SetCurveLineSizeMode(displayNode.UseLineDiameter)
        displayNode.SetLineDiameter(thickness)

        # Now safe to hide the point markers - the line no longer depends on this.
        displayNode.SetGlyphScale(0.0)
        displayNode.SetPointLabelsVisibility(False)
        displayNode.SetPropertiesLabelVisibility(True)