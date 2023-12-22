
(cl:in-package :asdf)

(defsystem "ltv_mpc-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "sample" :depends-on ("_package_sample"))
    (:file "_package_sample" :depends-on ("_package"))
    (:file "sample_lst" :depends-on ("_package_sample_lst"))
    (:file "_package_sample_lst" :depends-on ("_package"))
  ))