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

(in-package :mot-man)

(defun eef-link-name (side)
  (cut:var-value '?link
                 (first (prolog:prolog `(and (cram-robot-interfaces:robot ?robot)
                                             (cram-robot-interfaces:end-effector-link ?robot ,side ?link))))))

(defun base-link-name ()
  (cut:var-value '?link
                 (first (prolog:prolog `(and (cram-robot-interfaces:robot ?robot)
                                             (cram-robot-interfaces:robot-base-frame ?robot ?link))))))

(defun planning-group-name (side)
  (cut:var-value '?group
                 (first (prolog:prolog `(and (cram-robot-interfaces:robot ?robot)
                                             (cram-robot-interfaces:planning-group ?robot ,side ?group))))))

(defun arms ()
  (mapcar (lambda (bdg)
            (cut:var-value '?arm bdg))
          (cut:force-ll (prolog:prolog `(and (cram-robot-interfaces:robot ?robot)
                                             (cram-robot-interfaces:arm ?robot ?arm))))))

(defun arm-link-names (side)
  (cut:var-value '?links
                 (first (prolog:prolog
                          `(and (cram-robot-interfaces:robot ?robot)
                                (cram-robot-interfaces:arm-links ?robot ,side ?links))))))

(defun arm-joint-names (side)
  (cut:var-value '?joints
                 (first (prolog:prolog
                          `(and (cram-robot-interfaces:robot ?robot)
                                (cram-robot-interfaces:arm-joints ?robot ,side ?joints))))))

(defun joint-upper-limit (joint)
  (let* ((bdg (first (prolog:prolog
                       `(and (cram-robot-interfaces:robot ?robot)
                             (cram-robot-interfaces:joint-upper-limit ?robot ,joint ?value))))))
    (when bdg
      (cut:var-value '?value
                     bdg))))

(defun joint-lower-limit (joint)
  (let* ((bdg (first (prolog:prolog
                       `(and (cram-robot-interfaces:robot ?robot)
                             (cram-robot-interfaces:joint-lower-limit ?robot ,joint ?value))))))
    (when bdg
      (cut:var-value '?value
                     bdg))))

(defun joint-origin (joint)
  (let* ((bdg (first (prolog:prolog
                       `(and (cram-robot-interfaces:robot ?robot)
                             (cram-robot-interfaces:joint-origin ?robot ,joint ?value))))))
    (when bdg
      (cut:var-value '?value
                     bdg))))
(defun joint-type (joint)
  (let* ((bdg (first (prolog:prolog
                       `(and (cram-robot-interfaces:robot ?robot)
                             (cram-robot-interfaces:joint-type ?robot ,joint ?value))))))
    (when bdg
      (cut:var-value '?value
                     bdg))))
(defun joint-axis (joint)
  (let* ((bdg (first (prolog:prolog
                       `(and (cram-robot-interfaces:robot ?robot)
                             (cram-robot-interfaces:joint-axis ?robot ,joint ?value))))))
    (when bdg
      (cut:var-value '?value
                     bdg))))

(defun joint-parent (joint)
  (let* ((bdg (first (prolog:prolog
                       `(and (cram-robot-interfaces:robot ?robot)
                             (cram-robot-interfaces:joint-parent-link ?robot ,joint ?value))))))
    (when bdg
      (cut:var-value '?value
                     bdg))))

(defun joint-child (joint)
  (let* ((bdg (first (prolog:prolog
                       `(and (cram-robot-interfaces:robot ?robot)
                             (cram-robot-interfaces:joint-child-link ?robot ,joint ?value))))))
    (when bdg
      (cut:var-value '?value
                     bdg))))

(defun hand-link-names (side)
  (cut:var-value '?links
                 (first (prolog:prolog
                          `(and (cram-robot-interfaces:robot ?robot)
                                (cram-robot-interfaces:hand-links ?robot ,side ?links))))))

(defun arm-base-joint-names (side)
  (cut:var-value '?joints
                 (first (prolog:prolog
                          `(and (cram-robot-interfaces:robot ?robot)
                                (cram-robot-interfaces:arm-base-joints ?robot ,side ?joints))))))

(defun arm-tool-joint-names (side)
  (cut:var-value '?joints
                 (first (prolog:prolog
                          `(and (cram-robot-interfaces:robot ?robot)
                                (cram-robot-interfaces:arm-tool-joints ?robot ,side ?joints))))))

(defun arm-base-link-names (side)
  (cut:var-value '?links
                 (first (prolog:prolog
                          `(and (cram-robot-interfaces:robot ?robot)
                                (cram-robot-interfaces:arm-base-links ?robot ,side ?links))))))

(defun manipulation-joint-names (&key filter-fixed)
  (let* ((manipulation-joint-names
           (remove-duplicates
             (append
               (apply #'append
                      (mapcar (lambda (arm)
                                (arm-base-joint-names arm))
                              (arms)))
               (apply #'append
                      (mapcar (lambda (arm)
                                (arm-joint-names arm))
                              (arms)))
               (apply #'append
                      (mapcar (lambda (arm)
                                (arm-tool-joint-names arm))
                              (arms))))
             :test #'equal))
         (manipulation-joint-names
           (if filter-fixed
             (remove-if (lambda (name)
                          (equal (joint-type name) :fixed))
                        manipulation-joint-names)
             manipulation-joint-names)))
    manipulation-joint-names))

(defun joint-arm (joint-name)
  (let* ((arms (arms))
         (arms-with-joint (mapcar (lambda (arm)
                                    (when (find joint-name (arm-joint-names arm) :test #'equal)
                                      arm))
                                  arms))
         (arm-bases-with-joint (mapcar (lambda (arm)
                                         (when (find joint-name (arm-base-joint-names arm) :test #'equal)
                                           (list :base arm)))
                                       arms))
         (arm-tools-with-joint (mapcar (lambda (arm)
                                         (when (find joint-name (arm-tool-joint-names arm) :test #'equal)
                                           (list :tool arm)))
                                       arms))
         (places-with-joint (remove-duplicates (remove-if #'null (append arms-with-joint arm-bases-with-joint arm-tools-with-joint))
                                               :test #'equal)))
    (if (and (equal (length places-with-joint) 1) (typep (car places-with-joint) 'keyword))
      (car places-with-joint)
      places-with-joint)))

(defun object-names-in-hand (side)
  (let* ((objects-in-hand
           (cut:lazy-mapcar (lambda (bdgs)
                              (cut:with-vars-bound (?o) bdgs
                                    (desig:current-desig ?o)))
                            (prolog:prolog `(cram-plan-occasions-events:object-in-hand
                                               ?o ,side))))
        (object-names-in-hand
          (cut:force-ll
                (cut:lazy-mapcar (lambda (object)
                                   (string-upcase (desig:desig-prop-value object :name)))
                                 objects-in-hand))))
    object-names-in-hand))

