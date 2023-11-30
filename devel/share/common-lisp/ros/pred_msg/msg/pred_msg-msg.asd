
(cl:in-package :asdf)

(defsystem "pred_msg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "correct_data" :depends-on ("_package_correct_data"))
    (:file "_package_correct_data" :depends-on ("_package"))
    (:file "state_data" :depends-on ("_package_state_data"))
    (:file "_package_state_data" :depends-on ("_package"))
  ))