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

(defun giskard-yaml-execution (yaml thresholds timeout)
  (cl-giskard:send-yaml-action yaml thresholds)
  (if (< 0.001 timeout)
    (cpl-impl:pursue
      (cl-giskard:wait-for-action-result)
      (progn
        (roslisp:wait-duration timeout)
        (error 'bad-convergence)))
    (cl-giskard:wait-for-action-result))
  (make-instance 'mot-man:manipulation-result :all-ok t))

(defun splice-yaml (additional-vars soft-constraints)
  (let* ((preamble (get-yaml-preamble))
         (control-variables (get-control-variables))
         (kinematics (get-kinematics))
         (auxiliary-definitions (varlist->yaml additional-vars))
         (controllable-constraints (get-controllable-constraints additional-vars))
         (hard-constraints (get-hard-constraints))
         (yaml (stitch-yaml-string preamble control-variables kinematics
                                   auxiliary-definitions controllable-constraints
                                   soft-constraints hard-constraints)))
    yaml))

(defun splice-and-execute-yaml (additional-vars soft-constraints thresholds timeout)
  (let* ((yaml (splice-yaml additional-vars soft-constraints)))
    (giskard-yaml-execution yaml thresholds timeout)))

