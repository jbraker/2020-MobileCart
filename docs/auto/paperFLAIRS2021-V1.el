(TeX-add-style-hook
 "paperFLAIRS2021-V1"
 (lambda ()
   (TeX-add-to-alist 'LaTeX-provided-class-options
                     '(("article" "letterpaper")))
   (TeX-add-to-alist 'LaTeX-provided-package-options
                     '(("url" "hyphens") ("rotating" "figuresright") ("todonotes" "colorinlistoftodos") ("algorithm2e" "english" "algo2e" "algoruled" "vlined" "linesnumbered")))
   (add-to-list 'LaTeX-verbatim-macros-with-braces-local "url")
   (add-to-list 'LaTeX-verbatim-macros-with-braces-local "path")
   (add-to-list 'LaTeX-verbatim-macros-with-delims-local "url")
   (add-to-list 'LaTeX-verbatim-macros-with-delims-local "path")
   (TeX-run-style-hooks
    "latex2e"
    "article"
    "art10"
    "aaai20"
    "times"
    "helvet"
    "courier"
    "url"
    "graphicx"
    "amssymb"
    "amsmath"
    "bm"
    "graphics"
    "adjustbox"
    "tikz"
    "subfigure"
    "epstopdf"
    "siunitx"
    "rotating"
    "todonotes"
    "algorithm2e"
    "enumerate"
    "easyReview")
   (TeX-add-symbols
    "year"
    "UrlFont")
   (LaTeX-add-labels
    "sec:introduction"
    "sec:problemSetup"
    "eq:leaderDT"
    "fig:leaderFollowerSetup"
    "eq:robotModel1-DT"
    "eq:stateError"
    "sec:reinf-learn"
    "fig:fig-rl"
    "sec:RLSolution"
    "eq:costFunctional"
    "eq:valueFunction"
    "eq:tempraldiffeq"
    "eq:argMinControlAction"
    "eq:BellOpt"
    "eq:valueFunctionEstimated"
    "eq:modelFreePolicy"
    "fig:nnCritic"
    "eq:const"
    "eq:criticWeights"
    "alg:ModelFreeTracking"
    "sec:resultsExperiments"
    "fig:trajectoryRectilinearDistance"
    "fig:stateErrorRectilinearDistance"
    "fig:controlInputsRectilinearDistance"
    "fig:weightRectilinearDistance"
    "fig:performanceRectilinearTrajectory"
    "fig:trajectorySinewave"
    "fig:stateErrorSinewave"
    "fig:controlInputSinewave"
    "fig:weightSinewaveDistance"
    "fig:performanceSinewaveTrajectory"
    "sec:conclusion")
   (LaTeX-add-environments
    "assumption")
   (LaTeX-add-bibliographies
    "bib/refsSuruzWeb"
    "bib/refsMultiAgent"
    "bib/refsRoboticsJournals"
    "bib/refsRoboticsConferences"
    "bib/refsGenericControl"
    "bib/refsBooksTRTheses"
    "bib/refsReinforcementLearningADP"
    "bib/refsRL-Keshtkar"))
 :latex)

