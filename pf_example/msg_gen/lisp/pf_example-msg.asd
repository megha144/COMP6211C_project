
(cl:in-package :asdf)

(defsystem "pf_example-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "RangeMeasurement" :depends-on ("_package_RangeMeasurement"))
    (:file "_package_RangeMeasurement" :depends-on ("_package"))
  ))