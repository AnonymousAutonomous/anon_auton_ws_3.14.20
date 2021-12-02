
(cl:in-package :asdf)

(defsystem "eyes-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Autonomous" :depends-on ("_package_Autonomous"))
    (:file "_package_Autonomous" :depends-on ("_package"))
    (:file "Choreo" :depends-on ("_package_Choreo"))
    (:file "_package_Choreo" :depends-on ("_package"))
    (:file "Custom" :depends-on ("_package_Custom"))
    (:file "_package_Custom" :depends-on ("_package"))
    (:file "Generic" :depends-on ("_package_Generic"))
    (:file "_package_Generic" :depends-on ("_package"))
  ))