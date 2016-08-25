;;; Copyright (c) 2016, Mihai Pomarlan <blandc@informatik.uni-bremen.de>
;;; All rights reserved.
;;; 
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;; 
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Universitaet Bremen nor the names of its contributors may be used to
;;;       endorse or promote products derived from this software without
;;;       specific prior written permission.
;;; 
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :cram-giskard-manager)

(defun get-yaml-preamble ()
  (format nil "#
# Copyright (C) 2016 Mihai Pomarlan <blandc@cs.uni-bremen.de>
#
# This file is an automatically generated YAML specification for giskard.
#
# giskard is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
#

scope:
  # definition of some nice short-cuts
  - zero_v: {vector3: [0, 0, 0]}
  - unit_x: {vector3: [1, 0, 0]}
  - unit_y: {vector3: [0, 1, 0]}
  - unit_z: {vector3: [0, 0, 1]}~%"))

(defun get-control-variables ()
  (let* ((manipulation-joint-names (mot-man:manipulation-joint-names :filter-fixed t))
         (manipulation-joint-indices (alexandria:iota (length manipulation-joint-names)))
         (joint-var-strings (mapcar (lambda (name index)
                                      (format nil "  - ~a: {input-var: ~a}~%"
                                            name
                                            index))
                                    manipulation-joint-names manipulation-joint-indices)))
    (format nil "  # definition of joint input variables~%~{~a~}~%" joint-var-strings)))

(defun get-joint-variable-transform (name type axis)
  ;; TODO: add all joint types here; and some failure handling to catch, log, and propagate upwards
  ;; that an unrecognized type was found
  (let* ((translation (cond
                        ((equal axis nil)
                          "zero_v")
                        ((< (cl-transforms:v-dist (cl-transforms:make-3d-vector 1 0 0) axis) 0.00001)
                          (format nil "{scale-vector: [~a, unit_x]}" name))
                        ((< (cl-transforms:v-dist (cl-transforms:make-3d-vector 0 1 0) axis) 0.00001)
                          (format nil "{scale-vector: [~a, unit_y]}" name))
                        ((< (cl-transforms:v-dist (cl-transforms:make-3d-vector 0 0 1) axis) 0.00001)
                          (format nil "{scale-vector: [~a, unit_z]}" name))
                        (T
                          (format nil "{scale-vector: [~a, {vector3: [~a, ~a, ~a]}]}"
                                      name (cl-transforms:x axis) (cl-transforms:y axis) (cl-transforms:z axis)))))
         (axis (cond
                 ((equal axis nil)
                   "unit_x")
                 ((< (cl-transforms:v-dist (cl-transforms:make-3d-vector 1 0 0) axis) 0.00001)
                   "unit_x")
                 ((< (cl-transforms:v-dist (cl-transforms:make-3d-vector 0 1 0) axis) 0.00001)
                   "unit_y")
                 ((< (cl-transforms:v-dist (cl-transforms:make-3d-vector 0 0 1) axis) 0.00001)
                   "unit_z")
                 (T
                   (format nil "{vector3: [~a, ~a, ~a]}"
                               (cl-transforms:x axis) (cl-transforms:y axis) (cl-transforms:z axis))))))
    (ecase type
      (:fixed
        (list "unit_x" 0 "zero_v"))
      (:revolute
          (list axis name "zero_v"))
      (:prismatic
          (list "unit_x" 0 translation))
      (:continuous
          (list axis name "zero_v")))))

(defun get-fk-transform-varname (base-link last-joint)
  (format nil "tr_~a_to_~a" base-link last-joint))

(defun get-joint-transform-varname (joint-name)
  (format nil "~a_frame" joint-name))

(defun get-joint-related-parameter-internal (possible-parameter-names defined-joint-variables)
  (if possible-parameter-names
    (let* ((cr-par-name (car possible-parameter-names))
           (have-parameter (assoc cr-par-name defined-joint-variables :test #'equal)))
      (if have-parameter
        (format nil "~a" cr-par-name)
        (get-joint-related-parameter-internal (cdr possible-parameter-names) defined-joint-variables)))
    "0"))

(defun get-joint-related-parameter (joint-name aux-variable defined-joint-variables)
  (let* ((joint-type (string-downcase (format nil "~a" (mot-man:joint-type joint-name))))
         (joint-types (if (equal joint-type "continuous")
                        (list "continuous" "revolute")
                        (list joint-type)))
         (joint-place-specs (mot-man:joint-arm joint-name))
         (joint-place-specs (if (typep joint-place-specs 'list)
                              joint-place-specs
                              (list joint-place-specs)))
         (joint-arms (mapcar (lambda (place-spec)
                               (let* ((joint-arm (if (typep place-spec 'list)
                                                   (format nil "~a_~a" (string-downcase (format nil "~a" (first place-spec)))
                                                                       (second place-spec))
                                                   place-spec))
                                      (joint-arm (string-downcase (format nil "~a_arm" joint-arm))))
                                 joint-arm))
                             joint-place-specs))
         (joint-place-type (if (find :base joint-place-specs
                                     :test (lambda (ref el)
                                             (and (typep el 'list)
                                                  (equal (car el) ref))))
                             "arm_base_joint"
                             (if (find :tool joint-place-specs
                                       :test (lambda (ref el)
                                               (and (typep el 'list)
                                                    (equal (car el) ref))))
                               "arm_tool_joint"
                               "arm_joint")))
         (var-suffix (cond
                       ((equal aux-variable :joint-minimum-velocity)
                         "min_vel")
                       ((equal aux-variable :joint-maximum-velocity)
                         "max_vel")
                       ((equal aux-variable :joint-weight)
                         "weight")
                       (t nil)))
         (possible-parameter-names (when var-suffix
                                     (append
                                       (list (format nil "~a_~a" joint-name var-suffix))
                                       (mapcar (lambda (joint-arm)
                                                 (format nil "~a_~a" joint-arm var-suffix))
                                               joint-arms)
                                       (list (format nil "~a_~a" joint-place-type var-suffix))
                                       (mapcar (lambda (joint-type)
                                                 (format nil "~a_~a" joint-type var-suffix))
                                               joint-types)
                                       (list (format nil "default_~a" var-suffix))))))
    (get-joint-related-parameter-internal possible-parameter-names defined-joint-variables)))

(defun get-joint-description-string (name)
  (let* ((origin (mot-man:joint-origin name))
         (type (mot-man:joint-type name))
         (axis (mot-man:joint-axis name))
         (jvt (get-joint-variable-transform name type axis))
         (axis (first jvt))
         (angle (second jvt))
         (translation (third jvt))
         (qx (cl-transforms:x (cl-transforms:rotation origin)))
         (qy (cl-transforms:y (cl-transforms:rotation origin)))
         (qz (cl-transforms:z (cl-transforms:rotation origin)))
         (qt (cl-transforms:w (cl-transforms:rotation origin)))
         (x (cl-transforms:x (cl-transforms:translation origin)))
         (y (cl-transforms:y (cl-transforms:translation origin)))
         (z (cl-transforms:z (cl-transforms:translation origin))))
    (format nil "  - ~a:~%      frame-mul:~%        - frame: [{quaternion: [~a, ~a, ~a, ~a]}, {vector3: [~a ~a ~a]}]~%        - frame: [{axis-angle: [~a, ~a]}, ~a]~%"
                (get-joint-transform-varname name) qx qy qz qt x y z axis angle translation)))

(defun get-joint-descriptions ()
  (let* ((arm-joint-names (remove-duplicates (mot-man:manipulation-joint-names)
                                             :test #'equal))
         (joint-strings (mapcar #'get-joint-description-string
                                arm-joint-names)))
    (format nil "  #definition of joint transforms~%~{~a~}~%" joint-strings)))

(defun get-arm-fk-string (arm)
  (let* ((joint-names (append (mot-man:arm-base-joint-names arm)
                              (mot-man:arm-joint-names arm)
                              (mot-man:arm-tool-joint-names arm)))
         (first-joint (first joint-names))
         (arm-base-link (mot-man:joint-parent first-joint))
         (fk-vars (cons
                    (format nil "  - ~a: {frame-mul: [~a]}~%"
                                (get-fk-transform-varname arm-base-link first-joint)
                                (get-joint-transform-varname first-joint))
                    (mapcar (lambda (cr-joint prev-joint)
                              (format nil "  - ~a: {frame-mul: [~a, ~a]}~%"
                                          (get-fk-transform-varname arm-base-link cr-joint)
                                          (get-fk-transform-varname arm-base-link prev-joint)
                                          (get-joint-transform-varname cr-joint)))
                            (cdr joint-names)
                            joint-names))))
    (cons (format nil "  #~a arm~%" arm)
          fk-vars)))

(defun get-arm-fk-strings ()
  (let* ((arms (mot-man:arms))
         (arm-fk-strings (apply #'append
                                (mapcar #'get-arm-fk-string
                                        arms))))
    (format nil "  #definitions of arm FK~%~{~a~}~%" (remove-duplicates arm-fk-strings
                                                                        :test #'equal
                                                                        :from-end t))))

(defun get-kinematics ()
  (format nil "~a~%~a~%" (get-joint-descriptions)
                         (get-arm-fk-strings)))

(defun get-controllable-constraints (defined-joint-variables)
  (let* ((manipulation-joint-names (mot-man:manipulation-joint-names :filter-fixed t))
         (manipulation-joint-indices (alexandria:iota (length manipulation-joint-names)))
         (controllable-constraints (mapcar
                                     (lambda (joint-name joint-index)
                                       (format nil "  - controllable-constraint: [~a, ~a, ~a, ~a, ~a]~%"
                                                   (get-joint-related-parameter joint-name :joint-minimum-velocity defined-joint-variables)
                                                   (get-joint-related-parameter joint-name :joint-maximum-velocity defined-joint-variables)
                                                   (get-joint-related-parameter joint-name :joint-weight defined-joint-variables)
                                                   joint-index joint-name))
                                     manipulation-joint-names manipulation-joint-indices)))
    (format nil "controllable-constraints:~%~{~a~}~%" controllable-constraints)))

(defun get-hard-constraints ()
  (let* ((manipulation-joint-names (mot-man:manipulation-joint-names :filter-fixed t))
         (upper-limits (mapcar #'mot-man:joint-upper-limit manipulation-joint-names))
         (lower-limits (mapcar #'mot-man:joint-lower-limit manipulation-joint-names))
         (hard-constraints (mapcar
                             (lambda (name upper lower)
                               (when (and lower upper)
                                 (format nil "  - hard-constraint:~%      - {double-sub: [~a, ~a]}~%      - {double-sub: [~a, ~a]}~%      - ~a~%" lower name upper name name)))
                             manipulation-joint-names upper-limits lower-limits))
         (hard-constraints (remove-if #'null hard-constraints)))
    (format nil "hard-constraints:~%~{~a~}~%" hard-constraints)))

(defun varlist->yaml (varlist)
  (let* ((var-yaml-strings (mapcar (lambda (var-entry)
                                     (let* ((name (car var-entry))
                                            (value (cadr var-entry)))
                                       (format nil "  - ~a: ~a~%" name value)))
                                   varlist)))
    (format nil "~{~a~}~%" var-yaml-strings)))

(defun stitch-yaml-string (preamble control-variables kinematics auxiliary-definitions controllable-constraints soft-constraints hard-constraints)
  ;; TODO: add logging here: log the created YAML string. (Otherwise, log it in the execute-arm-action)
  (format nil "~a~%~a~%~a~%~a~%~a~%~a~%~a~%"
              preamble
              control-variables
              kinematics
              auxiliary-definitions
              controllable-constraints
              soft-constraints
              hard-constraints))

