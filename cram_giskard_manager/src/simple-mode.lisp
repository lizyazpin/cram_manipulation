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

;; Frequency with which giskard sends feedback messages (msgs/sec)
(defparameter feedback-frequency 10)
;; Period (in seconds) between resets of the integral component of the
;; low-level controllers
(defparameter int-clear-cycle 2)
;; Duration (in seconds) of the integral component reset
(defparameter int-reset-len 0.1)

(defparameter ticks-for-cycle (* feedback-frequency int-clear-cycle))
(defparameter ticks-for-reset (* feedback-frequency int-reset-len))

(defun tr->ps (tr)
  (cl-transforms-stamped:make-pose-stamped
    (cl-transforms-stamped:frame-id tr)
    0
    (cl-transforms-stamped:translation tr)
    (cl-transforms-stamped:rotation tr)))

(defun get-raise-elbow-goal (arm)
    ;; TODO: actually implement this mode
  (declare (ignore arm)))

(defun get-interpolated-goal (arm cr-goal)
    ;; TODO: actually implement this mode
  (declare (ignore arm) (ignore cr-goal)))

(defun giskard-simple-mode-execution (goal-spec)
  (let* ((keys (keys goal-spec))
         (raised-elbows (cadr (assoc :raise-elbow keys)))
         ;; TODO: the underlying giskard interface supports only left and right arms
         ;; hence the lack of genericity here. This might change in the future
         (left-eef-name (mot-man:eef-link-name :left))
         (right-eef-name (mot-man:eef-link-name :right))
         (robot-base-name (mot-man:base-link-name))
         (cr-tran-left (cl-tf:lookup-transform cram-moveit::*transformer* robot-base-name left-eef-name))
         (cr-tran-right (cl-tf:lookup-transform cram-moveit::*transformer* robot-base-name right-eef-name))
         (cr-pose-left (tr->ps cr-tran-left))
         (cr-pose-right (tr->ps cr-tran-right))
         (left-arm-goals (find :left (arm-pose-goals goal-spec)
                               :test (lambda (ref arg)
                                       (equal (side arg) ref))))
         (right-arm-goals (find :right (arm-pose-goals goal-spec)
                                :test (lambda (ref arg)
                                        (equal (side arg) ref))))
         (cr-goal-left (if (find :left raised-elbows)
                         (get-raise-elbow-goal :left)
                         (car left-arm-goals)))
         (cr-goal-right (if (find :right raised-elbows)
                          (get-raise-elbow-goal :right)
                          (car right-arm-goals)))
         (tick-count 0))
    ;; TODO: actually implement this mode
    (error 'bad-convergence)))
