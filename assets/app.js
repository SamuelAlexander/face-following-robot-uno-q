// SPDX-FileCopyrightText: Copyright (C) ARDUINO SRL (http://www.arduino.cc)
//
// SPDX-License-Identifier: MPL-2.0

const recentDetectionsElement = document.getElementById('recentDetections');
const feedbackContentElement = document.getElementById('feedback-content');
const MAX_RECENT_SCANS = 5;
let scans = [];
const socket = io(`http://${window.location.host}`);
let errorContainer = document.getElementById('error-container');
// Suppress socket.emit during initial slider setup to avoid resetting backend values.
let slidersReady = false;

// Start the application
document.addEventListener('DOMContentLoaded', () => {
    initSocketIO();
    initializeConfidenceSlider();
    initializeSteerGainSlider();
    initializeDriveBiasSlider();
    slidersReady = true;
    initializeEmergencyStop();
    initializeMotorTest();
    updateFeedback(null);
    renderDetections();

    // Popover logic
    const confidencePopoverText = "Minimum confidence score for detected objects. Lower values show more results but may include false positives.";
    const feedbackPopoverText = "When the camera detects a face, visual feedback will be shown here.";
    const steerGainPopoverText = "How aggressively the robot turns toward the target. Higher values turn faster. Adjust while watching the robot to find the sweet spot.";
    const driveBiasPopoverText = "Fixed forward/backward offset independent of object size. Negative moves backward, positive moves forward, zero keeps turn-only behavior.";

    document.querySelectorAll('.info-btn.confidence').forEach(img => {
        const popover = img.nextElementSibling;
        img.addEventListener('mouseenter', () => {
            popover.textContent = confidencePopoverText;
            popover.style.display = 'block';
        });
        img.addEventListener('mouseleave', () => {
            popover.style.display = 'none';
        });
    });

    document.querySelectorAll('.info-btn.feedback').forEach(img => {
        const popover = img.nextElementSibling;
        img.addEventListener('mouseenter', () => {
            popover.textContent = feedbackPopoverText;
            popover.style.display = 'block';
        });
        img.addEventListener('mouseleave', () => {
            popover.style.display = 'none';
        });
    });

    document.querySelectorAll('.info-btn.steerGain').forEach(img => {
        const popover = img.nextElementSibling;
        img.addEventListener('mouseenter', () => {
            popover.textContent = steerGainPopoverText;
            popover.style.display = 'block';
        });
        img.addEventListener('mouseleave', () => {
            popover.style.display = 'none';
        });
    });

    document.querySelectorAll('.info-btn.driveBias').forEach(img => {
        const popover = img.nextElementSibling;
        img.addEventListener('mouseenter', () => {
            popover.textContent = driveBiasPopoverText;
            popover.style.display = 'block';
        });
        img.addEventListener('mouseleave', () => {
            popover.style.display = 'none';
        });
    });
});

function initSocketIO() {
    socket.on('connect', () => {
        if (errorContainer) {
            errorContainer.style.display = 'none';
            errorContainer.textContent = '';
        }
    });

    socket.on('disconnect', () => {
        if (errorContainer) {
            errorContainer.textContent = 'Connection to the board lost. Please check the connection.';
            errorContainer.style.display = 'block';
        }
    });

    socket.on('detection', async (message) => {
        printDetection(message);
        renderDetections();
        updateFeedback(message);
    });

    socket.on('robot_state', (message) => {
        printRobotState(message);
        renderDetections();
    });

}

function updateFeedback(detection) {
    const objectInfo = {
        "face": { text: "Following you!", gif: "hand.gif" },
    };

    if (detection && objectInfo[detection.content]) {
        const info = objectInfo[detection.content];
        const confidence = Math.floor(detection.confidence * 100);
        feedbackContentElement.innerHTML = `
            <div class="feedback-detection">
                <div class="percentage">${confidence}%</div>
                <img src="img/${info.gif}" alt="${detection.content}">
                <p>${info.text}</p>
            </div>
        `;
    } else {
        feedbackContentElement.innerHTML = `
            <img src="img/stars.svg" alt="Stars">
            <p class="feedback-text">System response will appear here</p>
        `;
    }
}

function printDetection(newDetection) {
    scans.unshift({ ...newDetection, kind: 'detection' });
    if (scans.length > MAX_RECENT_SCANS) { scans.pop(); }
}

function printRobotState(stateMessage) {
    scans.unshift({ ...stateMessage, kind: 'robot_state' });
    if (scans.length > MAX_RECENT_SCANS) { scans.pop(); }
}

// Function to render the list of scans
function renderDetections() {
    // Clear the list
    recentDetectionsElement.innerHTML = ``;

    if (scans.length === 0) {
        recentDetectionsElement.innerHTML = `
            <div class="no-recent-scans">
                <img src="./img/no-face.svg">
                No object detected yet
            </div>
        `;
        return;
    }

    scans.forEach((scan) => {
        const row = document.createElement('div');
        row.className = 'scan-container';

        // Create a container for content and time
        const cellContainer = document.createElement('span');
        cellContainer.className = 'scan-cell-container cell-border';

        // Content (text + icon)
        const contentText = document.createElement('span');
        contentText.className = 'scan-content';
        if (scan.kind === 'robot_state') {
            const modeLabel = scan.mode ? ` (${scan.mode})` : '';
            const left = Number.isFinite(scan.left_us) ? scan.left_us : '-';
            const right = Number.isFinite(scan.right_us) ? scan.right_us : '-';
            const ls = Number.isFinite(scan.left_speed) ? scan.left_speed : '-';
            const rs = Number.isFinite(scan.right_speed) ? scan.right_speed : '-';
            const areaPct = Number.isFinite(scan.area_frac) ? `${Math.round(scan.area_frac * 100)}%` : '';
            const areaSuffix = areaPct ? ` | area:${areaPct}` : '';
            contentText.textContent = `Robot: ${scan.status}${modeLabel} | L:${left}(${ls}) R:${right}(${rs})${areaSuffix}`;
        } else {
            const value = scan.confidence;
            const result = Math.floor(value * 1000) / 10;
            const bboxStr = scan.bbox ? ` [${scan.bbox.map(v => Math.round(v)).join(',')}]` : '';
            contentText.innerHTML = `${result}% - ${scan.content}${bboxStr}`;
        }

        // Time
        const timeText = document.createElement('span');
        timeText.className = 'scan-content-time';
        timeText.textContent = new Date(scan.timestamp).toLocaleString('it-IT').replace(',', ' -');

        // Append content and time to the container
        cellContainer.appendChild(contentText);
        cellContainer.appendChild(timeText);

        row.appendChild(cellContainer);
        recentDetectionsElement.appendChild(row);
    });
}


function initializeConfidenceSlider() {
    const confidenceSlider = document.getElementById('confidenceSlider');
    const confidenceInput = document.getElementById('confidenceInput');
    const confidenceResetButton = document.getElementById('confidenceResetButton');

    confidenceSlider.addEventListener('input', updateConfidenceDisplay);
    confidenceInput.addEventListener('input', handleConfidenceInputChange);
    confidenceInput.addEventListener('blur', validateConfidenceInput);
    updateConfidenceDisplay();

    confidenceResetButton.addEventListener('click', (e) => {
        if (e.target.classList.contains('reset-icon') || e.target.closest('.reset-icon')) {
            resetConfidence();
        }
    });
}

function handleConfidenceInputChange() {
    const confidenceInput = document.getElementById('confidenceInput');
    const confidenceSlider = document.getElementById('confidenceSlider');

    let value = parseFloat(confidenceInput.value);

    if (isNaN(value)) value = 0.5;
    if (value < 0) value = 0;
    if (value > 1) value = 1;

    confidenceSlider.value = value;
    updateConfidenceDisplay();
}

function validateConfidenceInput() {
    const confidenceInput = document.getElementById('confidenceInput');
    let value = parseFloat(confidenceInput.value);

    if (isNaN(value)) value = 0.5;
    if (value < 0) value = 0;
    if (value > 1) value = 1;

    confidenceInput.value = value.toFixed(2);

    handleConfidenceInputChange();
}

function updateConfidenceDisplay() {
    const confidenceSlider = document.getElementById('confidenceSlider');
    const confidenceInput = document.getElementById('confidenceInput');
    const confidenceValueDisplay = document.getElementById('confidenceValueDisplay');
    const sliderProgress = document.getElementById('sliderProgress');

    const value = parseFloat(confidenceSlider.value);
    if (slidersReady) socket.emit('override_th', value);
    const percentage = (value - confidenceSlider.min) / (confidenceSlider.max - confidenceSlider.min) * 100;

    const displayValue = value.toFixed(2);
    confidenceValueDisplay.textContent = displayValue;

    if (document.activeElement !== confidenceInput) {
        confidenceInput.value = displayValue;
    }

    sliderProgress.style.width = percentage + '%';
    confidenceValueDisplay.style.left = percentage + '%';
}

function resetConfidence() {
    const confidenceSlider = document.getElementById('confidenceSlider');
    const confidenceInput = document.getElementById('confidenceInput');

    confidenceSlider.value = '0.5';
    confidenceInput.value = '0.50';
    updateConfidenceDisplay();
}

// --- Steer Gain slider -------------------------------------------------

function initializeSteerGainSlider() {
    const slider = document.getElementById('steerGainSlider');
    const input = document.getElementById('steerGainInput');
    const resetBtn = document.getElementById('steerGainResetButton');

    slider.addEventListener('input', updateSteerGainDisplay);
    input.addEventListener('input', handleSteerGainInputChange);
    input.addEventListener('blur', validateSteerGainInput);
    updateSteerGainDisplay();

    resetBtn.addEventListener('click', (e) => {
        if (e.target.classList.contains('reset-icon') || e.target.closest('.reset-icon')) {
            resetSteerGain();
        }
    });
}

function handleSteerGainInputChange() {
    const input = document.getElementById('steerGainInput');
    const slider = document.getElementById('steerGainSlider');

    let value = parseFloat(input.value);
    if (isNaN(value)) value = 0.15;
    if (value < 0) value = 0;
    if (value > 0.30) value = 0.30;

    slider.value = value;
    updateSteerGainDisplay();
}

function validateSteerGainInput() {
    const input = document.getElementById('steerGainInput');
    let value = parseFloat(input.value);

    if (isNaN(value)) value = 0.15;
    if (value < 0) value = 0;
    if (value > 0.30) value = 0.30;

    input.value = value.toFixed(3);
    handleSteerGainInputChange();
}

function updateSteerGainDisplay() {
    const slider = document.getElementById('steerGainSlider');
    const input = document.getElementById('steerGainInput');
    const valueDisplay = document.getElementById('steerGainValueDisplay');
    const progress = document.getElementById('steerGainSliderProgress');

    const value = parseFloat(slider.value);
    if (slidersReady) socket.emit('override_steer_gain', value);
    const percentage = (value - slider.min) / (slider.max - slider.min) * 100;

    const displayValue = value.toFixed(3);
    valueDisplay.textContent = displayValue;

    if (document.activeElement !== input) {
        input.value = displayValue;
    }

    progress.style.width = percentage + '%';
    valueDisplay.style.left = percentage + '%';
}

function resetSteerGain() {
    const slider = document.getElementById('steerGainSlider');
    const input = document.getElementById('steerGainInput');

    slider.value = '0.15';
    input.value = '0.15';
    updateSteerGainDisplay();
}

// --- Drive Bias slider --------------------------------------------------

function initializeDriveBiasSlider() {
    const slider = document.getElementById('driveBiasSlider');
    const input = document.getElementById('driveBiasInput');
    const resetBtn = document.getElementById('driveBiasResetButton');

    slider.addEventListener('input', updateDriveBiasDisplay);
    input.addEventListener('input', handleDriveBiasInputChange);
    input.addEventListener('blur', validateDriveBiasInput);
    updateDriveBiasDisplay();

    resetBtn.addEventListener('click', (e) => {
        if (e.target.classList.contains('reset-icon') || e.target.closest('.reset-icon')) {
            resetDriveBias();
        }
    });
}

function handleDriveBiasInputChange() {
    const input = document.getElementById('driveBiasInput');
    const slider = document.getElementById('driveBiasSlider');

    let value = parseFloat(input.value);
    if (isNaN(value)) value = 0.0;
    if (value < -0.08) value = -0.08;
    if (value > 0.08) value = 0.08;

    slider.value = value;
    updateDriveBiasDisplay();
}

function validateDriveBiasInput() {
    const input = document.getElementById('driveBiasInput');
    let value = parseFloat(input.value);

    if (isNaN(value)) value = 0.0;
    if (value < -0.08) value = -0.08;
    if (value > 0.08) value = 0.08;

    input.value = value.toFixed(3);
    handleDriveBiasInputChange();
}

function updateDriveBiasDisplay() {
    const slider = document.getElementById('driveBiasSlider');
    const input = document.getElementById('driveBiasInput');
    const valueDisplay = document.getElementById('driveBiasValueDisplay');
    const progress = document.getElementById('driveBiasSliderProgress');

    const value = parseFloat(slider.value);
    if (slidersReady) socket.emit('override_drive_bias', value);
    const percentage = (value - slider.min) / (slider.max - slider.min) * 100;

    const displayValue = value.toFixed(3);
    valueDisplay.textContent = displayValue;

    if (document.activeElement !== input) {
        input.value = displayValue;
    }

    // Bipolar slider: fill from center (50%) outward.
    const center = 50;
    if (percentage >= center) {
        progress.style.left = center + '%';
        progress.style.width = (percentage - center) + '%';
    } else {
        progress.style.left = percentage + '%';
        progress.style.width = (center - percentage) + '%';
    }
    valueDisplay.style.left = percentage + '%';
}

function resetDriveBias() {
    const slider = document.getElementById('driveBiasSlider');
    const input = document.getElementById('driveBiasInput');

    slider.value = '0';
    input.value = '0.000';
    updateDriveBiasDisplay();
}

// --- Emergency Stop --------------------------------------------------

function initializeEmergencyStop() {
    const btn = document.getElementById('emergencyStopBtn');
    let stopped = false;

    btn.addEventListener('click', () => {
        stopped = !stopped;
        if (stopped) {
            socket.emit('emergency_stop', true);
            btn.textContent = 'RESUME';
            btn.classList.add('stopped');
        } else {
            socket.emit('emergency_release', true);
            btn.textContent = 'STOP';
            btn.classList.remove('stopped');
        }
    });
}

// --- Motor Test ------------------------------------------------------

function initializeMotorTest() {
    const btn = document.getElementById('motorTestBtn');
    let running = false;

    btn.addEventListener('click', () => {
        if (!running) {
            running = true;
            btn.textContent = 'TESTING...';
            btn.classList.add('running');
            socket.emit('motor_test', true);
        }
    });

    socket.on('motor_test_done', () => {
        running = false;
        btn.textContent = 'MOTOR TEST';
        btn.classList.remove('running');
    });
}
