
(cl:in-package :asdf)

(defsystem "ltv_mpc-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :ltv_mpc-msg
)
  :components ((:file "_package")
    (:file "gp_srv" :depends-on ("_package_gp_srv"))
    (:file "_package_gp_srv" :depends-on ("_package"))
  ))