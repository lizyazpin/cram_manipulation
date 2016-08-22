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

(defclass giskard-goal-specification (manipulation-goal-specification)
  ())

(defmethod mot-man:make-goal-specification ((type (eql :giskard-goal-specification)) &rest args)
  (apply #'make-instance (cons 'giskard-goal-specification args)))

(defmethod mot-man:make-goal-specification ((type (eql 'giskard-goal-specification)) &rest args)
  (apply #'make-instance (cons 'giskard-goal-specification args)))

(defmethod mot-man:execute-arm-action ((goal-specification giskard-goal-specification))
  ;; TODO: add an interpretation of the goal spec, and appropriate giskard calls, here.
  ;; For now, just fall-back to whatever the fallback is
  (call-fallback goal-specification))

(defun fallback-to-giskard (goal-spec)
  (copy-goal-specification goal-spec 'giskard-goal-specification))

(defmethod make-fallback-converter ((type (eql :giskard-goal-specification)))
  #'fallback-to-giskard)

(defmethod make-fallback-converter ((type (eql 'giskard-goal-specification)))
  #'fallback-to-giskard)


