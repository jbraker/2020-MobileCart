(TeX-add-style-hook
 "BBBlueIntroC"
 (lambda ()
   (add-to-list 'LaTeX-verbatim-environments-local "minted")
   (add-to-list 'LaTeX-verbatim-macros-with-braces-local "path")
   (add-to-list 'LaTeX-verbatim-macros-with-braces-local "url")
   (add-to-list 'LaTeX-verbatim-macros-with-braces-local "nolinkurl")
   (add-to-list 'LaTeX-verbatim-macros-with-braces-local "hyperbaseurl")
   (add-to-list 'LaTeX-verbatim-macros-with-braces-local "hyperimage")
   (add-to-list 'LaTeX-verbatim-macros-with-braces-local "hyperref")
   (add-to-list 'LaTeX-verbatim-macros-with-braces-local "href")
   (add-to-list 'LaTeX-verbatim-macros-with-delims-local "path")
   (LaTeX-add-labels
    "sec:partsNeeded"
    "fig:BBBlue"
    "sec:putty"
    "sec:winSCP"
    "sec:etcher"
    "sec:flashingOS"
    "fig:Etcher"
    "fig:debianImageUpload"
    "fig:debianImageUploadSelectDrive"
    "fig:debianImageUploadSelectDriveFlashing"
    "fig:debianImageUpload2-MicroSD"
    "fig:PuTTY-Configuration"
    "fig:PuTTY-Console"
    "fig:PuTTY1"
    "sec:usbConn"
    "sec:drivers"
    "fig:BBBlue-Start"
    "fig:BBBlue-DriverInstalation"
    "fig:ifconfig_1"
    "fig:connmanctl"
    "sec:softwareInstallation"
    "sec:firstprogram"
    "fig:PuTTYLog"
    "fig:PuTTYcons"
    "fig:rcTestDriver"
    "fig:winSCPlogin"
    "fig:winApp"
    "sec:troubleshooting"
    "trst:problem2"
    "trst:problem4"
    "trst:problem1"
    "sec:useful-links")
   (LaTeX-add-environments
    '("prelab" LaTeX-env-args ["argument"] 1)))
 :latex)

